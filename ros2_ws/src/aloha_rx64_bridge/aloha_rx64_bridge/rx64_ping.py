import argparse

import rclpy
from rclpy.node import Node

from .rx64_driver import Rx64Driver, Rx64Error


class Rx64PingNode(Node):
    def __init__(self):
        super().__init__("rx64_ping")
        self.declare_parameter("device_name", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 57600)
        self.declare_parameter("servo_id", 6)

        self.device_name = self.get_parameter("device_name").value
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.servo_id = int(self.get_parameter("servo_id").value)


def main(args=None):
    parser = argparse.ArgumentParser(add_help=False)
    _, ros_args = parser.parse_known_args(args=args)

    rclpy.init(args=ros_args)
    node = Rx64PingNode()
    driver = Rx64Driver(node.device_name, node.baudrate, node.servo_id)

    try:
        driver.open()
        model_num = driver.ping()
        state = driver.read_state()
        node.get_logger().info(f"Ping success: model=0x{model_num:04X}, servo_id={node.servo_id}")
        node.get_logger().info(
            "State: "
            f"position_raw={state['position_raw']}, "
            f"speed_raw={state['speed_raw']}, "
            f"voltage={state['voltage_v']:.1f}V, "
            f"temperature={state['temperature_c']}C"
        )
    except Rx64Error as exc:
        node.get_logger().error(str(exc))
        raise SystemExit(1)
    finally:
        driver.close()
        node.destroy_node()
        rclpy.shutdown()
