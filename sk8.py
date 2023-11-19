import time
import asyncio
from typing import Callable, Optional, Any, Sequence
import struct

# imports from pyscript and the pyodide interpreter it uses
from pyscript import when
from pyodide.ffi import to_js, create_proxy, JsProxy
from pyodide.ffi.wrappers import add_event_listener, remove_event_listener

# wrappers around standard Javascript objects and methods
from js import (
    console,
    window,
    Object,
    setInterval,
    document,
    BluetoothDevice,
    CanvasRenderingContext2D,
)

# from constants.py in the original driver
IMU_DATA_STRUCT = struct.Struct("<hhhhhhhhhBB")
EXTANA_DATA_STRUCT = struct.Struct("<HHHB")
MAX_IMUS = 5

LED_MIN = 0
LED_MAX = 255.0
INT_LED_MAX = 3000.0

EXT_HW_NONE = 0x00
EXT_HW_IMUS = 0x01
EXT_HW_EXTANA = 0x02

SENSOR_ACC = 0x01
SENSOR_GYRO = 0x02
SENSOR_MAG = 0x04
SENSOR_ALL = SENSOR_ACC | SENSOR_GYRO | SENSOR_MAG

MAX_DEVICE_NAME_LEN = 20

UUID_BATTERY_LEVEL = "2a19"
UUID_DEVICE_NAME = "2a00"
UUID_FIRMWARE_REVISION = "2a26"
UUID_SK8_SERVICE = "b9e32260-1074-11e6-a7d5-0002a5d5c51b"
UUID_IMU_CCC = "b9e32261-1074-11e6-a7d5-0002a5d5c51b"
UUID_EXTANA_CCC = "b9e32262-1074-11e6-a7d5-0002a5d5c51b"
UUID_IMU_SELECTION = "b9e32263-1074-11e6-a7d5-0002a5d5c51b"
UUID_SENSOR_SELECTION = "b9e32264-1074-11e6-a7d5-0002a5d5c51b"
UUID_SOFT_RESET = "b9e32265-1074-11e6-a7d5-0002a5d5c51b"
UUID_EXTANA_LED = "b9e32266-1074-11e6-a7d5-0002a5d5c51b"
UUID_POLLING_OVERRIDE = "b9e3226a-1074-11e6-a7d5-0002a5d5c51b"

UUID_EXTANA_IMU_STREAMING = "b9e32286-1074-11e6-a7d5-0002a5d5c51b"
UUID_HARDWARE_STATE = "b9e32269-1074-11e6-a7d5-0002a5d5c51b"

RAW_UUID_GATT_PRIMARY_SERVICE = b"\x00\x28"
RAW_UUID_GATT_CHAR_DECL = b"\x03\x28"
RAW_UUID_GATT_CCC = b"\x02\x29"
UUID_GATT_CCC = "2902"
RAW_CCC_NOTIFY_ON = b"\x01\x00"
RAW_CCC_NOTIFY_OFF = b"\x00\x00"


# from imu.py in the original driver
class IMUData:
    """
    Instances of this class provide access to sensor data from individual IMUs.

    Attributes:
        acc (list): latest accelerometer readings, [x, y, z]
        mag (list): latest magnetometer readings, [x, y, z]
        acc (list): latest gyroscope readings, [x, y, z]
        seq (int): sequence number from most recent packet (0-255 range)
        timestamp (float): value of `time.time()` when packet received
    """

    PACKET_PERIOD = 3

    def __init__(self, index: int, calibration_data: Optional[dict[str, Any]] = None):
        self._calibration_data = calibration_data
        self.index = index
        self.reset()

    def reset(self):
        self.acc = (0.0, 0.0, 0.0)
        self.mag = (0, 0, 0)
        self.gyro = (0, 0, 0)
        self.acc_scale = (1.0, 1.0, 1.0)
        self.acc_offsets = (0, 0, 0)
        self.seq = 0
        self._use_calibration = False
        self._has_acc_calib, self.has_mag_calib, self.has_gyro_calib = (
            False,
            False,
            False,
        )
        self.load_calibration(self._calibration_data)
        self._packet_metadata = []
        self._packet_start = time.time()
        self._packets_lost = 0

    def get_sample_rate(self) -> float:
        if time.time() - self._packet_start < IMUData.PACKET_PERIOD:
            return -1
        return len(self._packet_metadata) / IMUData.PACKET_PERIOD

    def get_packets_lost(self) -> int:
        sample_rate = self.get_sample_rate()
        if sample_rate == -1:
            return -1

        return sum([x[1] for x in self._packet_metadata])

    def get_total_packets_lost(self) -> int:
        return self._packets_lost

    def set_calibration(self, state: bool) -> None:
        self._use_calibration = state

    def get_calibration(self) -> bool:
        return self._use_calibration

    def load_calibration(self, calibration_data: Optional[dict[str, Any]]) -> bool:
        axes = ["x", "y", "z"]
        if calibration_data is None:
            return False

        if "accx_offset" in calibration_data:
            self.acc_scale = tuple(
                map(float, [calibration_data[f"acc{x}_scale"] for x in axes])
            )
            self.acc_offsets = tuple(
                map(float, [calibration_data[f"acc{x}_offset"] for x in axes])
            )
            self.has_acc_calib = True
        else:
            self.acc_scale = (1.0, 1.0, 1.0)
            self.acc_offsets = (0, 0, 0)

        if "gyrox_offset" in calibration_data:
            self.gyro_offsets = tuple(
                map(float, [calibration_data[f"gyro{x}_offset"] for x in axes])
            )
            self.has_gyro_calib = True
        else:
            self.gyro_offsets = (0, 0, 0)

        if "magx_offset" in calibration_data:
            self.mag_scale = tuple(
                map(float, [calibration_data[f"mag{x}_scale"] for x in axes])
            )
            self.mag_offsets = tuple(
                map(float, [calibration_data[f"mag{x}_offset"] for x in axes])
            )
            self.has_mag_calib = True
        else:
            self.mag_offsets = (1.0, 1.0, 1.0)
            self.mag_scale = (0, 0, 0)

        self._use_calibration = True
        return True

    def _get_cal(
        self,
        raw: Sequence[float],
        offset: Sequence[float],
        scale: Sequence[float],
        ignore_cal: bool = False,
    ) -> Sequence[float]:
        if ignore_cal:
            return raw

        return tuple((raw[x] * scale[x]) - offset[x] for x in range(len(raw)))

    def update(
        self,
        acc: Sequence[float],
        gyro: Sequence[float],
        mag: Sequence[float],
        seq: int,
        timestamp: float,
    ) -> None:
        if not self._use_calibration:
            self.acc = acc
            self.gyro = gyro
            self.mag = mag
        else:
            self.acc = tuple(
                map(int, self._get_cal(acc, self.acc_offsets, self.acc_scale))
            )
            self.gyro = self._get_cal(gyro, self.gyro_offsets, (1.0, 1.0, 1.0))
            self.mag = tuple(
                map(int, self._get_cal(mag, self.mag_offsets, self.mag_scale))
            )

        dropped = 0
        if self.seq != -1:
            expected = (self.seq + 1) % 256
            if expected != seq:
                dropped = (expected - seq) % 256
                self._packets_lost += dropped
        self.seq = seq
        self.timestamp = timestamp
        self._packet_metadata.insert(0, (timestamp, dropped))
        now = time.time()
        while now - self._packet_metadata[-1][0] > IMUData.PACKET_PERIOD:
            self._packet_metadata.pop()

    def __repr__(self):
        return f"[{self.index}] acc={self.acc}, mag={self.mag}, gyro={self.gyro}, seq={self.seq}"


class SK8Pyscript:
    def __init__(self):
        self._gatt_device: Optional[BluetoothDevice] = None
        self._user_imu_callback = None
        self._user_imu_callback_data = None
        self._handle_cache = {}
        self._imus = [IMUData(x) for x in range(MAX_IMUS)]

    async def connect(self, device: BluetoothDevice) -> bool:
        """
        Create a connection to an SK8.

        <device> will be a BluetoothDevice object passed in by the WebBluetooth API (see connectCallback below)
        """
        await device.gatt.connect()
        console.log("> Connected to device", device, type(device))
        self._gatt_device = device
        return True

    def disconnect(self) -> bool:
        """
        Disconnect from an SK8.
        """
        if self._gatt_device is None:
            console.log("Can't disconnect, not currently connected")
            return False

        self._gatt_device.gatt.disconnect()
        self._handle_cache = {}  # clear any cached characteristic handles

        return True

    async def _get_handle(
        self, char_uuid: str, service_uuid: str = UUID_SK8_SERVICE
    ) -> JsProxy:
        """
        Utility method to retrieve a characteristic handle.
        """
        key = f"{service_uuid}_{char_uuid}"
        if key in self._handle_cache:
            return self._handle_cache[key]

        service = await self._gatt_device.gatt.getPrimaryService(service_uuid)
        char = await service.getCharacteristic(char_uuid)

        # these lookups can take a noticeable amount of time, so cache the handle
        # in case it needs repeated
        self._handle_cache[key] = char

        return char

    async def enable_imu_streaming(
        self, enabled_imus: list[int], enabled_sensors: int = SENSOR_ALL
    ) -> bool:
        """
        Start streaming data from one or more SK8 IMUs.

        <enabled_imus> is a list of IMU indexes from 0 -- (MAX_IMUS - 1). Only "0" is a valid
        value here if the SK8 doesn't have external IMUs attached.

        <enabled_sensors> is a bitmask indicating which sensors should be enabled on each IMU.
        It defaults to SENSOR_ALL, which means accelerometer + gyroscope + magnetometer.

        To modify this, use a combination of SENSOR_ACC/SENSOR_GYRO/SENSOR_MAG.
        """
        imus_enabled = 0
        for imu in enabled_imus:
            imus_enabled |= 1 << imu

        if enabled_sensors == 0:
            console.warn("Not enabling IMUs, no sensors enabled!")
            return False

        # first configure the the IMU state (IMUs enabled and sensors enabled on each active IMU)
        imu_select_char = await self._get_handle(UUID_IMU_SELECTION)
        console.log("> Got IMU char")
        sensor_select_char = await self._get_handle(UUID_SENSOR_SELECTION)
        console.log("> Got sensor char")

        # note need to call to_js to convert the Python bytes objects to the correct Javascript type
        await imu_select_char.writeValueWithoutResponse(
            to_js(struct.pack("B", imus_enabled))
        )
        await sensor_select_char.writeValueWithoutResponse(
            to_js(struct.pack("B", enabled_sensors))
        )

        imu_noti_char = await self._get_handle(UUID_IMU_CCC)

        # now activate notifications to begin streaming IMU data
        await imu_noti_char.startNotifications()
        console.log("> Enabled notifications!")

        # create an event listener to trigger it each time a new value arrives
        # (this method takes care of the Python-to-JS part for you since it's a pyodide wrapper)
        console.log("> Setting up event listener for notifications")
        add_event_listener(
            imu_noti_char, "characteristicvaluechanged", self._imu_callback
        )

        return True

    async def disable_imu_streaming(self) -> bool:
        """
        Disable IMU data streaming on the connected device.
        """
        imu_noti_char = await self._get_handle(UUID_IMU_CCC)

        console.log("> Removing event listener on IMU callback")
        remove_event_listener(
            imu_noti_char, "characteristicvaluechanged", self._imu_callback
        )
        await imu_noti_char.stopNotifications()

        return True

    def _imu_callback(self, event: JsProxy) -> None:
        """
        Internal handler for new streaming data packets.
        """

        # convert the JSProxy payload back to a Python object (in this case it'll be a memoryview)
        data = event.target.value.to_py()

        # now can parse contents as usual
        _data = IMU_DATA_STRUCT.unpack(data)
        acc, gyro, mag, imu, seq = (
            _data[:3],
            _data[3:6],
            _data[6:9],
            _data[9],
            _data[10],
        )
        self._imus[imu].update(acc, gyro, mag, seq, time.time())

        # call the registered user IMU callback, if any
        if self._user_imu_callback is not None:
            self._user_imu_callback(imu, self._imus[imu], self._user_imu_callback_data)

    def set_imu_callback(self, callback: Optional[Callable], user_data: Any = None):
        """
        Register a callable to be triggered when new sensor data packets arrive.

        The callable should have a signature of:
            name(imu_index: int, imu_object: IMUData, user_data: Any)
        """
        self._user_imu_callback = callback
        self._user_imu_callback_data = user_data

    def enable_extana_streaming(
        self, include_imu: bool = False, enabled_sensors: int = SENSOR_ALL
    ):
        raise Exception("Not implemented yet")

    def disable_extana_streaming(self):
        raise Exception("Not implemented yet")

    def get_device_name(self):
        raise Exception("Not implemented yet")

    def get_battery_level(self):
        raise Exception("Not implemented yet")

    def get_firmware_version(self):
        raise Exception("Not implemented yet")

    def get_imu(self, imu_number: int):
        """
        Return an IMUData instance for the given IMU index.

        The index must be between 0 and (MAX_IMUS - 1). If no external IMUs are
        connected, the only populated entry will be index 0. IMUs are numbered
        in order of increasing distance along the chain from the SK8 itself.
        """
        if imu_number < 0 or imu_number >= MAX_IMUS:
            raise Exception(f"Invalid imu_number, valid values are [0, {MAX_IMUS}]")

        return self._imus[imu_number]

    def get_extana(self):
        raise Exception("Not implemented yet")


# global vars to retain objects
sk8 = SK8Pyscript()
graph = None


def userImuCallback(imu_index: int, imu: IMUData, data: None):
    """
    Called by the SK8 class when new sensor data is received
    """

    # print(imu.acc)
    if graph is not None:
        graph.update([imu.acc[i] for i in range(3)])


async def device_connect(device) -> None:
    console.log("> Connecting to device")
    await sk8.connect(device)
    document.querySelector("#connectButton").disabled = True
    document.querySelector("#disconnectButton").disabled = False
    document.querySelector("#startNotifications").disabled = False
    document.querySelector("#stopNotifications").disabled = False


@when("click", "#connectButton")
def connectCallback(event) -> None:
    """
    Handler for clicking the Connect button
    """

    # suppress the default reload-page action
    event.preventDefault()

    # define a dict with the structure expected by the WebBluetooth requestDevice method,
    # using the service UUID to define a device filter so we only see SK8s listed
    requestOptions = {"filters": [{"services": [UUID_SK8_SERVICE]}]}

    # convert the Python dict into the JS object the method is actually expecting
    jsopts = to_js(requestOptions, dict_converter=Object.fromEntries)

    # prompt the user to select a device. once they've made a selection, the
    # connection process continues in device_connect
    window.navigator.bluetooth.requestDevice(jsopts).then(device_connect)



@when("click", "#disconnectButton")
def disconnectCallback(event):
    """
    Handler for clicking the Disconnect button
    """

    # suppress the default reload-page action
    event.preventDefault()

    sk8.disconnect()
    document.querySelector("#connectButton").disabled = False
    document.querySelector("#disconnectButton").disabled = True
    document.querySelector("#startNotifications").disabled = True
    document.querySelector("#stopNotifications").disabled = True


@when("click", "#startNotifications")
def startNotifications(event):
    """
    Handles clicks on the "Start streaming" button
    """

    # suppress the default reload-page action
    event.preventDefault()

    sk8.set_imu_callback(userImuCallback)

    # can't make these handlers async def, so using ensure_future
    # to run the methods as an asyncio task
    asyncio.ensure_future(sk8.enable_imu_streaming([0]))


@when("click", "#stopNotifications")
def stopNotifications(event):
    """
    Handles clicks on the "Stop streaming" button
    """
    # suppress the default reload-page action
    event.preventDefault()

    # can't make these handlers async def, so using ensure_future
    # to run the methods as an asyncio task
    asyncio.ensure_future(sk8.disable_imu_streaming())


class SimpleLineGraph:
    """
    Very simple line graph widget for showing sensor data.

    This is using Python (pyodide) to interact with a Javascript/HTML canvas object.
    """

    def __init__(
        self, ctx: CanvasRenderingContext2D, width: int, height: int, points=100
    ) -> None:
        self._ctx = ctx
        self._width = width
        self._height = height
        self._data = [
            [0.0 for _ in range(points)],
            [0.0 for _ in range(points)],
            [0.0 for _ in range(points)],
        ]
        self._colours = ["red", "green", "blue"]
        self._points = points
        self._y_scale = 2000  # TODO other sensors
        self._graph_scale = self._height / (2.0 * self._y_scale)

    def _scale_point(self, val: float) -> float:
        return (val + self._y_scale) * self._graph_scale

    def _draw_series(self, values, hscale) -> None:
        self._ctx.beginPath()
        self._ctx.moveTo(0, values[0])
        for i, value in enumerate(values[1:]):
            self._ctx.lineTo((i + 1) * hscale, value)
        self._ctx.stroke()

    def _draw_line(self, points) -> None:
        self._ctx.beginPath()
        self._ctx.moveTo(*points[0])
        for point in points[1:]:
            self._ctx.lineTo(*point)
        self._ctx.stroke()

    def update(self, newdata: list[float]) -> None:
        """
        Update data series.

        Expects one new value in the list for each of the 3 data streams.
        """
        for i in range(len(self._data)):
            self._data[i].append(self._scale_point(newdata[i]))
            self._data[i].pop(0)

    def draw(self, _) -> None:
        self._ctx.clearRect(0, 0, self._width, self._height)
        self._ctx.lineWidth = 1
        self._ctx.strokeStyle = "#ccc"

        # (top left is 0, 0)
        # draw a line showing the x-axis
        self._draw_line([(0, self._height / 2), (self._width, self._height / 2)])

        for i in range(len(self._data)):
            if len(self._data[i]) == 0:
                continue

            self._ctx.strokeStyle = self._colours[i]
            self._draw_series(self._data[i], self._width / len(self._data[i]))


def main():
    global graph
    canvas = document.getElementById("canvas")
    ctx = canvas.getContext("2d")

    graph = SimpleLineGraph(ctx, canvas.width, canvas.height)
    setInterval(create_proxy(graph.draw), 50, ctx)


console.log("Loaded")
main()
