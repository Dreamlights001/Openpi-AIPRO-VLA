import math

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler


ADDR_TORQUE_ENABLE = 24
ADDR_GOAL_POSITION = 30
ADDR_MOVING_SPEED = 32
ADDR_PRESENT_POSITION = 36
ADDR_PRESENT_SPEED = 38
ADDR_PRESENT_VOLTAGE = 42
ADDR_PRESENT_TEMPERATURE = 43

RX64_PROTOCOL_VERSION = 1.0
RX64_POSITION_MAX = 1023
RX64_TRAVEL_DEG = 300.0
RX64_TRAVEL_RAD = math.radians(RX64_TRAVEL_DEG)


class Rx64Error(RuntimeError):
    """Raised when Dynamixel communication fails."""


class Rx64Driver:
    def __init__(self, device_name: str, baudrate: int, servo_id: int):
        self.device_name = device_name
        self.baudrate = int(baudrate)
        self.servo_id = int(servo_id)
        self.port_handler = PortHandler(device_name)
        self.packet_handler = PacketHandler(RX64_PROTOCOL_VERSION)
        self._is_open = False

    def open(self) -> None:
        if self._is_open:
            return
        if not self.port_handler.openPort():
            raise Rx64Error(f"Failed to open port: {self.device_name}")
        if not self.port_handler.setBaudRate(self.baudrate):
            self.port_handler.closePort()
            raise Rx64Error(f"Failed to set baudrate: {self.baudrate}")
        self._is_open = True

    def close(self) -> None:
        if self._is_open:
            self.port_handler.closePort()
            self._is_open = False

    def ping(self) -> int:
        model_num, result, error = self.packet_handler.ping(self.port_handler, self.servo_id)
        self._check_result("ping", result, error)
        return model_num

    def enable_torque(self, enabled: bool) -> None:
        self.write_u8(ADDR_TORQUE_ENABLE, 1 if enabled else 0)

    def set_moving_speed(self, speed_raw: int) -> None:
        self.write_u16(ADDR_MOVING_SPEED, int(max(0, min(1023, speed_raw))))

    def set_goal_position_raw(self, position_raw: int) -> None:
        self.write_u16(ADDR_GOAL_POSITION, int(max(0, min(RX64_POSITION_MAX, position_raw))))

    def read_present_position_raw(self) -> int:
        return self.read_u16(ADDR_PRESENT_POSITION)

    def read_present_speed_raw(self) -> int:
        return self.read_u16(ADDR_PRESENT_SPEED)

    def read_present_voltage(self) -> float:
        return self.read_u8(ADDR_PRESENT_VOLTAGE) * 0.1

    def read_present_temperature(self) -> int:
        return self.read_u8(ADDR_PRESENT_TEMPERATURE)

    def read_state(self) -> dict:
        return {
            "position_raw": self.read_present_position_raw(),
            "speed_raw": self.read_present_speed_raw(),
            "voltage_v": self.read_present_voltage(),
            "temperature_c": self.read_present_temperature(),
        }

    def write_u8(self, address: int, value: int) -> None:
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, self.servo_id, address, int(value)
        )
        self._check_result(f"write_u8({address})", result, error)

    def write_u16(self, address: int, value: int) -> None:
        result, error = self.packet_handler.write2ByteTxRx(
            self.port_handler, self.servo_id, address, int(value)
        )
        self._check_result(f"write_u16({address})", result, error)

    def read_u8(self, address: int) -> int:
        value, result, error = self.packet_handler.read1ByteTxRx(
            self.port_handler, self.servo_id, address
        )
        self._check_result(f"read_u8({address})", result, error)
        return int(value)

    def read_u16(self, address: int) -> int:
        value, result, error = self.packet_handler.read2ByteTxRx(
            self.port_handler, self.servo_id, address
        )
        self._check_result(f"read_u16({address})", result, error)
        return int(value)

    def raw_to_rad(self, position_raw: int, zero_position_raw: int = 512) -> float:
        return ((int(position_raw) - int(zero_position_raw)) / RX64_POSITION_MAX) * RX64_TRAVEL_RAD

    def rad_to_raw(
        self,
        position_rad: float,
        zero_position_raw: int = 512,
        min_position_raw: int = 0,
        max_position_raw: int = RX64_POSITION_MAX,
    ) -> int:
        raw = int(round(int(zero_position_raw) + (float(position_rad) / RX64_TRAVEL_RAD) * RX64_POSITION_MAX))
        return max(int(min_position_raw), min(int(max_position_raw), raw))

    def deg_to_raw(
        self,
        position_deg: float,
        zero_position_raw: int = 512,
        min_position_raw: int = 0,
        max_position_raw: int = RX64_POSITION_MAX,
    ) -> int:
        return self.rad_to_raw(
            math.radians(position_deg),
            zero_position_raw=zero_position_raw,
            min_position_raw=min_position_raw,
            max_position_raw=max_position_raw,
        )

    def _check_result(self, action: str, result: int, error: int) -> None:
        if result != COMM_SUCCESS:
            raise Rx64Error(f"{action} failed: {self.packet_handler.getTxRxResult(result)}")
        if error != 0:
            raise Rx64Error(f"{action} failed: {self.packet_handler.getRxPacketError(error)}")
