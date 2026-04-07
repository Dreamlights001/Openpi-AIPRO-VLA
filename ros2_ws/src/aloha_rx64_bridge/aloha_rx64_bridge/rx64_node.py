from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, UInt16
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory

from .rx64_driver import Rx64Driver, Rx64Error


class Rx64SingleNode(Node):
    def __init__(self):
        super().__init__("rx64_single")

        self.declare_parameter("device_name", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 57600)
        self.declare_parameter("servo_id", 6)
        self.declare_parameter("joint_name", "rx64_joint")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("moving_speed", 100)
        self.declare_parameter("zero_position_raw", 512)
        self.declare_parameter("min_position_raw", 0)
        self.declare_parameter("max_position_raw", 1023)
        self.declare_parameter("torque_on_startup", True)

        self.device_name = self.get_parameter("device_name").value
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.servo_id = int(self.get_parameter("servo_id").value)
        self.joint_name = self.get_parameter("joint_name").value
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.moving_speed = int(self.get_parameter("moving_speed").value)
        self.zero_position_raw = int(self.get_parameter("zero_position_raw").value)
        self.min_position_raw = int(self.get_parameter("min_position_raw").value)
        self.max_position_raw = int(self.get_parameter("max_position_raw").value)
        self.torque_on_startup = bool(self.get_parameter("torque_on_startup").value)

        self.driver = Rx64Driver(self.device_name, self.baudrate, self.servo_id)
        self._last_position_raw: Optional[int] = None

        self.driver.open()
        model_num = self.driver.ping()
        self.get_logger().info(
            f"Connected RX-64 on {self.device_name}, servo_id={self.servo_id}, model=0x{model_num:04X}"
        )
        self.driver.set_moving_speed(self.moving_speed)
        if self.torque_on_startup:
            self.driver.enable_torque(True)

        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(Float64, "~/goal_position_rad", self.goal_position_rad_callback, 10)
        self.create_subscription(Float64, "~/goal_position_deg", self.goal_position_deg_callback, 10)
        self.create_subscription(UInt16, "~/goal_raw", self.goal_raw_callback, 10)
        self.create_subscription(JointTrajectory, "~/command", self.command_callback, 10)
        self.create_service(SetBool, "~/torque_enable", self.torque_enable_callback)

        timer_period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.create_timer(timer_period, self.publish_joint_state)

    def goal_position_rad_callback(self, msg: Float64) -> None:
        raw = self.driver.rad_to_raw(
            msg.data,
            zero_position_raw=self.zero_position_raw,
            min_position_raw=self.min_position_raw,
            max_position_raw=self.max_position_raw,
        )
        self._write_goal(raw, f"{msg.data:.4f} rad")

    def goal_position_deg_callback(self, msg: Float64) -> None:
        raw = self.driver.deg_to_raw(
            msg.data,
            zero_position_raw=self.zero_position_raw,
            min_position_raw=self.min_position_raw,
            max_position_raw=self.max_position_raw,
        )
        self._write_goal(raw, f"{msg.data:.2f} deg")

    def goal_raw_callback(self, msg: UInt16) -> None:
        raw = max(self.min_position_raw, min(self.max_position_raw, int(msg.data)))
        self._write_goal(raw, f"raw={raw}")

    def command_callback(self, msg: JointTrajectory) -> None:
        if not msg.points:
            self.get_logger().warning("Received JointTrajectory with no points")
            return
        if self.joint_name not in msg.joint_names:
            self.get_logger().warning(
                f"JointTrajectory does not contain configured joint_name={self.joint_name}"
            )
            return
        joint_index = msg.joint_names.index(self.joint_name)
        point = msg.points[0]
        if joint_index >= len(point.positions):
            self.get_logger().warning("JointTrajectory point has no position for configured joint")
            return
        target_rad = point.positions[joint_index]
        raw = self.driver.rad_to_raw(
            target_rad,
            zero_position_raw=self.zero_position_raw,
            min_position_raw=self.min_position_raw,
            max_position_raw=self.max_position_raw,
        )
        self._write_goal(raw, f"JointTrajectory {target_rad:.4f} rad")

    def torque_enable_callback(self, request: SetBool.Request, response: SetBool.Response):
        try:
            self.driver.enable_torque(request.data)
            response.success = True
            response.message = "torque enabled" if request.data else "torque disabled"
        except Rx64Error as exc:
            response.success = False
            response.message = str(exc)
            self.get_logger().error(response.message)
        return response

    def publish_joint_state(self) -> None:
        try:
            state = self.driver.read_state()
            self._last_position_raw = state["position_raw"]

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = [self.joint_name]
            msg.position = [
                self.driver.raw_to_rad(
                    state["position_raw"],
                    zero_position_raw=self.zero_position_raw,
                )
            ]
            self.joint_state_pub.publish(msg)
        except Rx64Error as exc:
            self.get_logger().error(f"Failed to read joint state: {exc}")

    def destroy_node(self):
        try:
            self.driver.close()
        finally:
            super().destroy_node()

    def _write_goal(self, raw: int, source: str) -> None:
        try:
            self.driver.set_goal_position_raw(raw)
            self.get_logger().info(f"Goal set from {source}, raw={raw}")
        except Rx64Error as exc:
            self.get_logger().error(f"Failed to write goal position: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Rx64SingleNode()
        rclpy.spin(node)
    except Rx64Error as exc:
        if node is None:
            print(f"RX-64 startup failed: {exc}")
        else:
            node.get_logger().error(f"RX-64 startup failed: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
