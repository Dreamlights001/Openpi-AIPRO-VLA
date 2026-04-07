#!/usr/bin/env python3
"""
RX-64 手动调节程序

支持两种模式:
1. keyboard: 终端键盘控制，不依赖额外库
2. gamepad : Linux 手柄事件设备控制，依赖 evdev

默认硬件参数:
- device: /dev/ttyUSB0
- protocol: 1.0
- baudrate: 57600
- servo id: 6

键盘模式默认按键:
- a / 左箭头: 向前减小位置
- d / 右箭头: 向后增大位置
- q: 大步减小位置
- e: 大步增大位置
- z: 去最小位
- x: 回中位
- c: 去最大位
- [: 速度减小
- ]: 速度增大
- s: 打印状态
- t: 开关扭矩
- h: 显示帮助
- Ctrl+C: 退出

手柄模式默认映射:
- 左摇杆 X 或十字键左右: 调位置
- RT / R2: 大步正向
- LT / L2: 大步反向
- A: 打印状态
- B: 开关扭矩
- X: 回中位
- Y: 显示帮助
"""

import argparse
import select
import sys
import termios
import time
import tty
from dataclasses import dataclass
from typing import Optional

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


@dataclass
class ServoStatus:
    position: int
    speed: int
    load: int
    voltage: float
    temp: int


class Rx64Controller:
    def __init__(self, device_name: str, baudrate: int, servo_id: int):
        self.device_name = device_name
        self.baudrate = int(baudrate)
        self.servo_id = int(servo_id)
        self.port = PortHandler(self.device_name)
        self.packet = PacketHandler(PROTOCOL)
        self.current_speed = 120
        self.torque_enabled = False

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
        self.torque_enabled = bool(enabled)

    def toggle_torque(self) -> bool:
        self.enable_torque(not self.torque_enabled)
        return self.torque_enabled

    def set_joint_mode(self) -> None:
        self.write_word(ADDR_CW_ANGLE_LIMIT, POSITION_MIN)
        self.write_word(ADDR_CCW_ANGLE_LIMIT, POSITION_MAX)

    def set_speed(self, speed: int) -> int:
        self.current_speed = max(0, min(1023, int(speed)))
        self.write_word(ADDR_MOVING_SPEED, self.current_speed)
        return self.current_speed

    def set_position(self, position_raw: int) -> int:
        target = max(POSITION_MIN, min(POSITION_MAX, int(position_raw)))
        self.write_word(ADDR_GOAL_POSITION, target)
        return target

    def read_status(self) -> ServoStatus:
        return ServoStatus(
            position=self.read_word(ADDR_PRESENT_POSITION),
            speed=self.read_word(ADDR_PRESENT_SPEED),
            load=self.read_word(ADDR_PRESENT_LOAD),
            voltage=self.read_byte(ADDR_PRESENT_VOLTAGE) * 0.1,
            temp=self.read_byte(ADDR_PRESENT_TEMP),
        )


def raw_to_deg(position_raw: int) -> float:
    return position_raw * 300.0 / 1023.0


def clamp(value: int, low: int, high: int) -> int:
    return max(low, min(high, value))


def print_status(prefix: str, status: ServoStatus) -> None:
    print(
        f"{prefix} pos={status.position:4d} ({raw_to_deg(status.position):7.2f} deg) | "
        f"speed={status.speed:4d} | load={status.load:4d} | voltage={status.voltage:.1f}V | temp={status.temp}C"
    )


class ManualSession:
    def __init__(
        self,
        controller: Rx64Controller,
        step_raw: int,
        big_step_raw: int,
        min_position: int,
        max_position: int,
        initial_speed: int,
    ):
        self.controller = controller
        self.step_raw = max(1, int(step_raw))
        self.big_step_raw = max(self.step_raw, int(big_step_raw))
        self.min_position = clamp(int(min_position), POSITION_MIN, POSITION_MAX)
        self.max_position = clamp(int(max_position), POSITION_MIN, POSITION_MAX)
        if self.min_position > self.max_position:
            self.min_position, self.max_position = self.max_position, self.min_position
        self.target_position: Optional[int] = None
        self.controller.set_speed(initial_speed)

    def initialize(self) -> None:
        status = self.controller.read_status()
        self.target_position = clamp(status.position, self.min_position, self.max_position)
        print_status("当前状态", status)
        print(f"当前目标位: {self.target_position}")

    def move_relative(self, delta: int) -> None:
        if self.target_position is None:
            self.initialize()
        self.target_position = clamp(self.target_position + int(delta), self.min_position, self.max_position)
        actual = self.controller.set_position(self.target_position)
        print(f"目标位置 -> {actual} ({raw_to_deg(actual):.2f} deg)")

    def move_absolute(self, position_raw: int) -> None:
        self.target_position = clamp(position_raw, self.min_position, self.max_position)
        actual = self.controller.set_position(self.target_position)
        print(f"目标位置 -> {actual} ({raw_to_deg(actual):.2f} deg)")

    def change_speed(self, delta: int) -> None:
        speed = self.controller.set_speed(self.controller.current_speed + int(delta))
        print(f"速度 -> {speed}")

    def print_live_status(self) -> None:
        print_status("反馈状态", self.controller.read_status())

    def toggle_torque(self) -> None:
        enabled = self.controller.toggle_torque()
        print(f"扭矩 -> {'开启' if enabled else '关闭'}")

    def print_help(self) -> None:
        print("\n键盘模式:")
        print("  a / 左箭头  : 小步减小位置")
        print("  d / 右箭头  : 小步增大位置")
        print("  q           : 大步减小位置")
        print("  e           : 大步增大位置")
        print("  z / x / c   : 最小位 / 中位 / 最大位")
        print("  [ / ]       : 速度减小 / 增大")
        print("  s           : 打印状态")
        print("  t           : 开关扭矩")
        print("  h           : 显示帮助")
        print("  Ctrl+C      : 退出\n")

    def handle_keyboard_key(self, key: str) -> bool:
        if key in ("a", "\x1b[D"):
            self.move_relative(-self.step_raw)
        elif key in ("d", "\x1b[C"):
            self.move_relative(self.step_raw)
        elif key == "q":
            self.move_relative(-self.big_step_raw)
        elif key == "e":
            self.move_relative(self.big_step_raw)
        elif key == "z":
            self.move_absolute(self.min_position)
        elif key == "x":
            self.move_absolute((self.min_position + self.max_position) // 2)
        elif key == "c":
            self.move_absolute(self.max_position)
        elif key == "[":
            self.change_speed(-20)
        elif key == "]":
            self.change_speed(20)
        elif key == "s":
            self.print_live_status()
        elif key == "t":
            self.toggle_torque()
        elif key == "h":
            self.print_help()
        elif key in ("g",):
            print(f"目标位置 = {self.target_position}")
        elif key in ("\x03",):
            return False
        return True


class RawTerminal:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = None

    def __enter__(self):
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)
        return self

    def __exit__(self, exc_type, exc, tb):
        if self.old_settings is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

    def read_key(self, timeout: float = 0.1) -> Optional[str]:
        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        if not ready:
            return None
        ch = sys.stdin.read(1)
        if ch == "\x1b":
            ready, _, _ = select.select([sys.stdin], [], [], 0.01)
            if ready:
                second = sys.stdin.read(1)
                if second == "[":
                    ready, _, _ = select.select([sys.stdin], [], [], 0.01)
                    if ready:
                        third = sys.stdin.read(1)
                        return "\x1b[" + third
                return ch + second
        return ch


def run_keyboard_mode(session: ManualSession) -> None:
    session.print_help()
    with RawTerminal() as terminal:
        while True:
            key = terminal.read_key(timeout=0.2)
            if key is None:
                continue
            keep_running = session.handle_keyboard_key(key)
            if not keep_running:
                break


def list_input_devices() -> None:
    try:
        from evdev import InputDevice, list_devices
    except ImportError as exc:
        raise RuntimeError("未安装 evdev，请先执行: python3 -m pip install --user evdev") from exc

    devices = [InputDevice(path) for path in list_devices()]
    if not devices:
        print("没有发现 /dev/input/event* 设备")
        return
    for device in devices:
        print(f"{device.path} | name={device.name} | phys={device.phys}")


def run_gamepad_mode(session: ManualSession, device_path: Optional[str], deadzone: int) -> None:
    try:
        from evdev import InputDevice, categorize, ecodes, list_devices
    except ImportError as exc:
        raise RuntimeError("未安装 evdev，请先执行: python3 -m pip install --user evdev") from exc

    if device_path is None:
        devices = [InputDevice(path) for path in list_devices()]
        if not devices:
            raise RuntimeError("未找到手柄输入设备，请先确认 /dev/input/event*")
        device = devices[0]
    else:
        device = InputDevice(device_path)

    print(f"使用手柄设备: {device.path} | {device.name}")
    print("手柄模式已启动，按 Ctrl+C 退出")
    print("A:状态  B:扭矩  X:中位  Y:帮助  左右方向/摇杆:调位置  LT/RT:大步")

    abs_state = {}
    last_axis_move = 0.0

    for event in device.read_loop():
        if event.type == ecodes.EV_KEY and event.value == 1:
            key_event = categorize(event)
            code = key_event.scancode

            if code in (ecodes.BTN_SOUTH,):
                session.print_live_status()
            elif code in (ecodes.BTN_EAST,):
                session.toggle_torque()
            elif code in (ecodes.BTN_WEST,):
                session.move_absolute((session.min_position + session.max_position) // 2)
            elif code in (ecodes.BTN_NORTH,):
                session.print_help()
            elif code in (ecodes.BTN_TR,):
                session.move_relative(session.big_step_raw)
            elif code in (ecodes.BTN_TL,):
                session.move_relative(-session.big_step_raw)
            elif code in (ecodes.BTN_DPAD_LEFT,):
                session.move_relative(-session.step_raw)
            elif code in (ecodes.BTN_DPAD_RIGHT,):
                session.move_relative(session.step_raw)

        elif event.type == ecodes.EV_ABS:
            abs_event = categorize(event)
            abs_state[abs_event.event.code] = abs_event.event.value

            now = time.time()
            if now - last_axis_move < 0.12:
                continue

            if abs_event.event.code == ecodes.ABS_HAT0X:
                if abs_event.event.value < 0:
                    session.move_relative(-session.step_raw)
                    last_axis_move = now
                elif abs_event.event.value > 0:
                    session.move_relative(session.step_raw)
                    last_axis_move = now
            elif abs_event.event.code == ecodes.ABS_X:
                if abs_event.event.value <= 255:
                    center = 128
                    axis_deadzone = min(deadzone, 24)
                else:
                    center = 32768
                    axis_deadzone = deadzone
                delta = abs(abs_event.event.value - center)
                if delta < axis_deadzone:
                    continue
                if abs_event.event.value < center:
                    session.move_relative(-session.step_raw)
                else:
                    session.move_relative(session.step_raw)
                last_axis_move = now
            elif abs_event.event.code in (ecodes.ABS_Z, ecodes.ABS_BRAKE):
                trigger_deadzone = min(deadzone, 30) if abs_event.event.value <= 255 else deadzone
                if abs_event.event.value > trigger_deadzone:
                    session.move_relative(-session.big_step_raw)
                    last_axis_move = now
            elif abs_event.event.code in (ecodes.ABS_RZ, ecodes.ABS_GAS):
                trigger_deadzone = min(deadzone, 30) if abs_event.event.value <= 255 else deadzone
                if abs_event.event.value > trigger_deadzone:
                    session.move_relative(session.big_step_raw)
                    last_axis_move = now


def parse_args():
    parser = argparse.ArgumentParser(description="RX-64 键盘/手柄手动调节程序")
    parser.add_argument("--device", default=DEVICENAME, help="舵机串口设备")
    parser.add_argument("--baudrate", type=int, default=BAUDRATE, help="波特率")
    parser.add_argument("--id", dest="servo_id", type=int, default=SERVO_ID, help="舵机 ID")
    parser.add_argument("--mode", choices=["keyboard", "gamepad"], default="keyboard", help="控制模式")
    parser.add_argument("--gamepad-device", default=None, help="手柄事件设备，例如 /dev/input/event4")
    parser.add_argument("--list-inputs", action="store_true", help="列出可用输入设备并退出")
    parser.add_argument("--step", type=int, default=16, help="小步进 raw 值")
    parser.add_argument("--big-step", type=int, default=64, help="大步进 raw 值")
    parser.add_argument("--speed", type=int, default=120, help="初始速度")
    parser.add_argument("--min-position", type=int, default=POSITION_MIN, help="允许的最小位置")
    parser.add_argument("--max-position", type=int, default=POSITION_MAX, help="允许的最大位置")
    parser.add_argument("--deadzone", type=int, default=8000, help="手柄模拟轴死区")
    parser.add_argument("--skip-confirm", action="store_true", help="跳过人工确认")
    return parser.parse_args()


def main():
    args = parse_args()

    if args.list_inputs:
        list_input_devices()
        return

    print("=" * 72)
    print("RX-64 手动调节程序")
    print("=" * 72)
    print(f"mode         : {args.mode}")
    print(f"device       : {args.device}")
    print(f"baudrate     : {args.baudrate}")
    print(f"servo_id     : {args.servo_id}")
    print(f"step         : {args.step}")
    print(f"big_step     : {args.big_step}")
    print(f"speed        : {args.speed}")
    print(f"min_position : {args.min_position}")
    print(f"max_position : {args.max_position}")
    print("=" * 72)

    if not args.skip_confirm:
        answer = input("确认机械结构安全，再输入 yes 继续: ").strip().lower()
        if answer != "yes":
            print("用户取消")
            return

    controller = Rx64Controller(args.device, args.baudrate, args.servo_id)

    try:
        controller.open()
        model_num = controller.ping()
        print(f"已连接舵机: ID={args.servo_id}, model=0x{model_num:04X}")

        controller.set_joint_mode()
        controller.set_speed(args.speed)
        controller.enable_torque(True)

        session = ManualSession(
            controller=controller,
            step_raw=args.step,
            big_step_raw=args.big_step,
            min_position=args.min_position,
            max_position=args.max_position,
            initial_speed=args.speed,
        )
        session.initialize()

        if args.mode == "keyboard":
            run_keyboard_mode(session)
        else:
            run_gamepad_mode(session, args.gamepad_device, args.deadzone)

    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，退出控制")
    except Exception as exc:
        print(f"\n错误: {exc}")
        sys.exit(1)
    finally:
        try:
            controller.enable_torque(False)
        except Exception:
            pass
        try:
            controller.close()
        except Exception:
            pass
        print("串口已关闭，扭矩已尝试关闭")


if __name__ == "__main__":
    main()
