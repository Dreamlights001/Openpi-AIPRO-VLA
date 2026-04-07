#!/usr/bin/env python3
"""
ROS2 bridge for a single Dynamixel RX-64.

This script is intentionally close to the known-good standalone control script:
it keeps the same register map and serial access pattern, but exposes ROS2
topics/services so the servo can be controlled from the ROS graph without
requiring a colcon package build first.
"""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, UInt16
from std_srvs.srv import SetBool

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler


DEVICENAME = "/dev/ttyUSB0"
PROTOCOL = 1.0
BAUDRATE = 57600
SERVO_ID = 6

ADDR_TORQUE_ENABLE = 24
ADDR_CW_ANGLE_LIMIT = 6
ADDR_CCW_ANGLE_LIMIT = 8
ADDR_GOAL_POSITION = 30
ADDR_MOVING_SPEED = 32
ADDR_PRESENT_POSITION = 36
ADDR_PRESENT_SPEED = 38
ADDR_PRESENT_LOAD = 40
ADDR_PRESENT_VOLTAGE = 42
ADDR_PRESENT_TEMP = 43

POSITION_MAX = 1023
TRAVEL_DEG = 300.0
TRAVEL_RAD = math.radians(TRAVEL_DEG)


class Rx64Ros2Node(Node):
    def __init__(self):
        super().__init__("rx64_bridge")

        self.declare_parameter("device_name", DEVICENAME)
        self.declare_parameter("baudrate", BAUDRATE)
        self.declare_parameter("servo_id", SERVO_ID)
        self.declare_parameter("default_speed", 200)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("joint_name", "rx64_joint")
        self.declare_parameter("zero_raw", 512)
        self.declare_parameter("auto_enable_torque", True)
        self.declare_parameter("joint_mode_on_startup", True)

        self.device_name = str(self.get_parameter("device_name").value)
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.servo_id = int(self.get_parameter("servo_id").value)
        self.default_speed = int(self.get_parameter("default_speed").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.joint_name = str(self.get_parameter("joint_name").value)
        self.zero_raw = int(self.get_parameter("zero_raw").value)
        self.auto_enable_torque = bool(self.get_parameter("auto_enable_torque").value)
        self.joint_mode_on_startup = bool(self.get_parameter("joint_mode_on_startup").value)

        self.port_handler = PortHandler(self.device_name)
        self.packet_handler = PacketHandler(PROTOCOL)
        self.last_goal_raw: Optional[int] = None

        self._open_port()
        self._ping()

        if self.joint_mode_on_startup:
            self.set_joint_mode()
        if self.auto_enable_torque:
            self.enable_torque(True)

        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(Float64, "/rx64/goal_position_deg", self.goal_deg_callback, 10)
        self.create_subscription(Float64, "/rx64/goal_position_rad", self.goal_rad_callback, 10)
        self.create_subscription(UInt16, "/rx64/goal_position_raw", self.goal_raw_callback, 10)
        self.create_subscription(UInt16, "/rx64/moving_speed", self.speed_callback, 10)
        self.create_service(SetBool, "/rx64/torque_enable", self.torque_callback)

        period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.create_timer(period, self.publish_joint_state)

        self.get_logger().info(
            "RX-64 ROS2 bridge ready: "
            f"device={self.device_name}, id={self.servo_id}, baudrate={self.baudrate}"
        )

    def _open_port(self) -> None:
        if not self.port_handler.openPort():
            raise RuntimeError(f"Failed to open serial port: {self.device_name}")
        if not self.port_handler.setBaudRate(self.baudrate):
            self.port_handler.closePort()
            raise RuntimeError(f"Failed to set baudrate: {self.baudrate}")

    def _ping(self) -> None:
        model_num, result, error = self.packet_handler.ping(self.port_handler, self.servo_id)
        self._check_result("ping", result, error)
        self.get_logger().info(f"Connected servo: id={self.servo_id}, model=0x{model_num:04X}")

    def _check_result(self, action: str, result: int, error: int) -> None:
        if result != COMM_SUCCESS:
            raise RuntimeError(f"{action} failed: {self.packet_handler.getTxRxResult(result)}")
        if error != 0:
            raise RuntimeError(f"{action} failed: {self.packet_handler.getRxPacketError(error)}")

    def read_byte(self, addr: int) -> int:
        value, result, error = self.packet_handler.read1ByteTxRx(self.port_handler, self.servo_id, addr)
        self._check_result(f"read_byte({addr})", result, error)
        return int(value)

    def read_word(self, addr: int) -> int:
        value, result, error = self.packet_handler.read2ByteTxRx(self.port_handler, self.servo_id, addr)
        self._check_result(f"read_word({addr})", result, error)
        return int(value)

    def write_byte(self, addr: int, value: int) -> None:
        result, error = self.packet_handler.write1ByteTxRx(self.port_handler, self.servo_id, addr, int(value))
        self._check_result(f"write_byte({addr})", result, error)

    def write_word(self, addr: int, value: int) -> None:
        result, error = self.packet_handler.write2ByteTxRx(self.port_handler, self.servo_id, addr, int(value))
        self._check_result(f"write_word({addr})", result, error)

    def enable_torque(self, enabled: bool) -> None:
        self.write_byte(ADDR_TORQUE_ENABLE, 1 if enabled else 0)

    def set_joint_mode(self) -> None:
        self.write_word(ADDR_CW_ANGLE_LIMIT, 0)
        self.write_word(ADDR_CCW_ANGLE_LIMIT, 1023)

    def set_position(self, position_raw: int, speed: Optional[int] = None) -> None:
        target_speed = self.default_speed if speed is None else int(speed)
        target_raw = max(0, min(POSITION_MAX, int(position_raw)))
        target_speed = max(0, min(1023, target_speed))
        self.write_word(ADDR_MOVING_SPEED, target_speed)
        self.write_word(ADDR_GOAL_POSITION, target_raw)
        self.last_goal_raw = target_raw
        self.get_logger().info(f"Goal written: raw={target_raw}, speed={target_speed}")

    def read_status(self) -> dict:
        return {
            "position_raw": self.read_word(ADDR_PRESENT_POSITION),
            "speed_raw": self.read_word(ADDR_PRESENT_SPEED),
            "load_raw": self.read_word(ADDR_PRESENT_LOAD),
            "voltage_v": self.read_byte(ADDR_PRESENT_VOLTAGE) * 0.1,
            "temp_c": self.read_byte(ADDR_PRESENT_TEMP),
        }

    def raw_to_deg(self, position_raw: int) -> float:
        return float(position_raw) * TRAVEL_DEG / POSITION_MAX

    def raw_to_rad(self, position_raw: int) -> float:
        return math.radians(self.raw_to_deg(position_raw))

    def deg_to_raw(self, position_deg: float) -> int:
        clamped_deg = max(0.0, min(TRAVEL_DEG, float(position_deg)))
        return int(round(clamped_deg * POSITION_MAX / TRAVEL_DEG))

    def rad_to_raw(self, position_rad: float) -> int:
        return self.deg_to_raw(math.degrees(float(position_rad)))

    def goal_deg_callback(self, msg: Float64) -> None:
        try:
            self.set_position(self.deg_to_raw(msg.data))
        except RuntimeError as exc:
            self.get_logger().error(str(exc))

    def goal_rad_callback(self, msg: Float64) -> None:
        try:
            self.set_position(self.rad_to_raw(msg.data))
        except RuntimeError as exc:
            self.get_logger().error(str(exc))

    def goal_raw_callback(self, msg: UInt16) -> None:
        try:
            self.set_position(int(msg.data))
        except RuntimeError as exc:
            self.get_logger().error(str(exc))

    def speed_callback(self, msg: UInt16) -> None:
        try:
            self.default_speed = max(0, min(1023, int(msg.data)))
            self.get_logger().info(f"Default speed updated: {self.default_speed}")
        except RuntimeError as exc:
            self.get_logger().error(str(exc))

    def torque_callback(self, request: SetBool.Request, response: SetBool.Response):
        try:
            self.enable_torque(request.data)
            response.success = True
            response.message = "torque enabled" if request.data else "torque disabled"
        except RuntimeError as exc:
            response.success = False
            response.message = str(exc)
            self.get_logger().error(response.message)
        return response

    def publish_joint_state(self) -> None:
        try:
            status = self.read_status()
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = [self.joint_name]
            msg.position = [self.raw_to_rad(status["position_raw"])]
            self.joint_state_pub.publish(msg)
        except RuntimeError as exc:
            self.get_logger().error(f"publish_joint_state failed: {exc}")

    def destroy_node(self):
        try:
            self.port_handler.closePort()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Rx64Ros2Node()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
