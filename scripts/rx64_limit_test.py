#!/usr/bin/env python3
"""
RX-64 前后方向最大位限测试脚本

用途:
1. 从当前位置开始，小步移动到前向位限
2. 再小步移动到后向位限
3. 每一步打印当前位置、负载、电压、温度
4. 最后回到中位，方便继续调试

默认参数基于当前现场已确认配置:
- device: /dev/ttyUSB0
- protocol: 1.0
- baudrate: 57600
- servo id: 6

注意:
- 这是“探测脚本”，不是量产控制程序
- 第一次运行建议先把 `--forward-target` / `--backward-target` 改保守一点
- 机械结构如果有干涉，立即 Ctrl+C
"""

import argparse
import sys
import time

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

POSITION_MIN = 0
POSITION_MAX = 1023
POSITION_MID = 512


class Rx64Tester:
    def __init__(self, device_name: str, baudrate: int, servo_id: int):
        self.device_name = device_name
        self.baudrate = int(baudrate)
        self.servo_id = int(servo_id)
        self.port = PortHandler(self.device_name)
        self.packet = PacketHandler(PROTOCOL)

    def open(self) -> None:
        if not self.port.openPort():
            raise RuntimeError(f"无法打开串口: {self.device_name}")
        if not self.port.setBaudRate(self.baudrate):
            self.port.closePort()
            raise RuntimeError(f"无法设置波特率: {self.baudrate}")

    def close(self) -> None:
        self.port.closePort()

    def _check(self, action: str, result: int, error: int) -> None:
        if result != COMM_SUCCESS:
            raise RuntimeError(f"{action} 失败: {self.packet.getTxRxResult(result)}")
        if error != 0:
            raise RuntimeError(f"{action} 失败: {self.packet.getRxPacketError(error)}")

    def ping(self) -> int:
        model_num, result, error = self.packet.ping(self.port, self.servo_id)
        self._check("ping", result, error)
        return int(model_num)

    def read_byte(self, address: int) -> int:
        value, result, error = self.packet.read1ByteTxRx(self.port, self.servo_id, address)
        self._check(f"read1({address})", result, error)
        return int(value)

    def read_word(self, address: int) -> int:
        value, result, error = self.packet.read2ByteTxRx(self.port, self.servo_id, address)
        self._check(f"read2({address})", result, error)
        return int(value)

    def write_byte(self, address: int, value: int) -> None:
        result, error = self.packet.write1ByteTxRx(self.port, self.servo_id, address, int(value))
        self._check(f"write1({address})", result, error)

    def write_word(self, address: int, value: int) -> None:
        result, error = self.packet.write2ByteTxRx(self.port, self.servo_id, address, int(value))
        self._check(f"write2({address})", result, error)

    def enable_torque(self, enabled: bool) -> None:
        self.write_byte(ADDR_TORQUE_ENABLE, 1 if enabled else 0)

    def set_joint_mode(self) -> None:
        self.write_word(ADDR_CW_ANGLE_LIMIT, POSITION_MIN)
        self.write_word(ADDR_CCW_ANGLE_LIMIT, POSITION_MAX)

    def set_speed(self, speed: int) -> None:
        self.write_word(ADDR_MOVING_SPEED, max(0, min(1023, int(speed))))

    def set_position(self, position_raw: int) -> None:
        self.write_word(ADDR_GOAL_POSITION, max(POSITION_MIN, min(POSITION_MAX, int(position_raw))))

    def read_status(self) -> dict:
        return {
            "position": self.read_word(ADDR_PRESENT_POSITION),
            "speed": self.read_word(ADDR_PRESENT_SPEED),
            "load": self.read_word(ADDR_PRESENT_LOAD),
            "voltage": self.read_byte(ADDR_PRESENT_VOLTAGE) * 0.1,
            "temp": self.read_byte(ADDR_PRESENT_TEMP),
        }


def raw_to_deg(position_raw: int) -> float:
    return round(position_raw * 300.0 / 1023.0, 2)


def print_status(prefix: str, status: dict) -> None:
    print(
        f"{prefix} pos={status['position']:4d} ({raw_to_deg(status['position']):7.2f} deg) | "
        f"speed={status['speed']:4d} | load={status['load']:4d} | "
        f"voltage={status['voltage']:.1f}V | temp={status['temp']}C"
    )


def move_and_wait(
    tester: Rx64Tester,
    target: int,
    settle_time: float,
    tolerance: int,
    poll_interval: float,
    timeout: float,
) -> dict:
    tester.set_position(target)
    deadline = time.time() + timeout
    last_status = tester.read_status()
    while time.time() < deadline:
        time.sleep(poll_interval)
        last_status = tester.read_status()
        if abs(last_status["position"] - target) <= tolerance:
            break
    time.sleep(settle_time)
    return tester.read_status()


def probe_direction(
    tester: Rx64Tester,
    name: str,
    start_pos: int,
    target_pos: int,
    step_raw: int,
    settle_time: float,
    tolerance: int,
    poll_interval: float,
    timeout: float,
    temp_limit: int,
) -> int:
    print(f"\n=== 开始测试{name}方向: {start_pos} -> {target_pos} ===")

    if start_pos == target_pos:
        status = tester.read_status()
        print_status("当前位置", status)
        return status["position"]

    direction = 1 if target_pos > start_pos else -1
    current_target = start_pos
    final_position = start_pos

    while True:
        next_target = current_target + direction * step_raw
        if direction > 0:
            next_target = min(next_target, target_pos)
        else:
            next_target = max(next_target, target_pos)

        status = move_and_wait(
            tester=tester,
            target=next_target,
            settle_time=settle_time,
            tolerance=tolerance,
            poll_interval=poll_interval,
            timeout=timeout,
        )
        print_status(f"{name}目标 {next_target:4d} ->", status)
        final_position = status["position"]

        if status["temp"] >= temp_limit:
            print(f"温度达到 {status['temp']}C，停止{name}方向测试")
            break

        current_target = next_target
        if current_target == target_pos:
            break

    print(f"=== {name}方向测试结束，最终反馈位置: {final_position} ({raw_to_deg(final_position):.2f} deg) ===")
    return final_position


def parse_args():
    parser = argparse.ArgumentParser(description="测试 RX-64 前后方向最大位限")
    parser.add_argument("--device", default=DEVICENAME, help="串口设备")
    parser.add_argument("--baudrate", type=int, default=BAUDRATE, help="波特率")
    parser.add_argument("--id", dest="servo_id", type=int, default=SERVO_ID, help="舵机 ID")
    parser.add_argument("--speed", type=int, default=120, help="测试速度，建议先保守")
    parser.add_argument("--step", type=int, default=64, help="每次步进的 raw 值")
    parser.add_argument("--forward-target", type=int, default=POSITION_MIN, help="前向极限目标")
    parser.add_argument("--backward-target", type=int, default=POSITION_MAX, help="后向极限目标")
    parser.add_argument("--return-target", type=int, default=POSITION_MID, help="测试结束返回位置")
    parser.add_argument("--settle", type=float, default=0.3, help="每步额外稳定时间")
    parser.add_argument("--poll", type=float, default=0.05, help="轮询反馈间隔")
    parser.add_argument("--timeout", type=float, default=2.0, help="每步等待超时")
    parser.add_argument("--tolerance", type=int, default=12, help="到位容差")
    parser.add_argument("--temp-limit", type=int, default=65, help="温度保护阈值")
    parser.add_argument(
        "--skip-confirm",
        action="store_true",
        help="跳过开始前人工确认",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    tester = Rx64Tester(args.device, args.baudrate, args.servo_id)

    print("=" * 72)
    print("RX-64 前后方向最大位限测试")
    print("=" * 72)
    print(f"device          : {args.device}")
    print(f"baudrate        : {args.baudrate}")
    print(f"servo_id        : {args.servo_id}")
    print(f"speed           : {args.speed}")
    print(f"step            : {args.step}")
    print(f"forward_target  : {args.forward_target}")
    print(f"backward_target : {args.backward_target}")
    print(f"return_target   : {args.return_target}")
    print("=" * 72)

    if not args.skip_confirm:
        answer = input("确认机械结构不会卡死，再输入 yes 继续: ").strip().lower()
        if answer != "yes":
            print("用户取消")
            return

    try:
        tester.open()
        model_num = tester.ping()
        print(f"已连接舵机: ID={args.servo_id}, model=0x{model_num:04X}")

        tester.set_joint_mode()
        tester.set_speed(args.speed)
        tester.enable_torque(True)
        time.sleep(0.2)

        start_status = tester.read_status()
        print_status("起始状态", start_status)

        front_feedback = probe_direction(
            tester=tester,
            name="前向",
            start_pos=start_status["position"],
            target_pos=max(POSITION_MIN, min(POSITION_MAX, args.forward_target)),
            step_raw=max(1, args.step),
            settle_time=args.settle,
            tolerance=max(0, args.tolerance),
            poll_interval=max(0.01, args.poll),
            timeout=max(0.1, args.timeout),
            temp_limit=args.temp_limit,
        )

        back_feedback = probe_direction(
            tester=tester,
            name="后向",
            start_pos=front_feedback,
            target_pos=max(POSITION_MIN, min(POSITION_MAX, args.backward_target)),
            step_raw=max(1, args.step),
            settle_time=args.settle,
            tolerance=max(0, args.tolerance),
            poll_interval=max(0.01, args.poll),
            timeout=max(0.1, args.timeout),
            temp_limit=args.temp_limit,
        )

        print("\n=== 返回中位 ===")
        return_status = move_and_wait(
            tester=tester,
            target=max(POSITION_MIN, min(POSITION_MAX, args.return_target)),
            settle_time=args.settle,
            tolerance=max(0, args.tolerance),
            poll_interval=max(0.01, args.poll),
            timeout=max(0.1, args.timeout),
        )
        print_status("返回完成", return_status)

        print("\n=== 测试结果汇总 ===")
        print(f"前向反馈极限: {front_feedback} ({raw_to_deg(front_feedback):.2f} deg)")
        print(f"后向反馈极限: {back_feedback} ({raw_to_deg(back_feedback):.2f} deg)")

    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，停止测试")
    except Exception as exc:
        print(f"\n错误: {exc}")
        sys.exit(1)
    finally:
        try:
            tester.enable_torque(False)
        except Exception:
            pass
        try:
            tester.close()
        except Exception:
            pass
        print("串口已关闭，扭矩已尝试关闭")


if __name__ == "__main__":
    main()
