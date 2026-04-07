# Dynamixel RX-64 电机调试与硬件配置文档 (Orange Pi AI Pro)

## 1. 系统环境
- **主机名**: `orangepiaipro`
- **用户**: `HwHiAiUser` (已加入 `dialout` 组，免 sudo 访问串口)
- **系统**: Ubuntu Linux (aarch64/arm64 架构)
- **Conda 环境**: `lerobot` (Python 执行需在此环境下)
- **工作目录**: `~/samples/ROS2`
- **字符编码**: `en_US.UTF-8` (系统英文，完美兼容中文 UTF-8)

## 2. USB 与串口硬件映射
- **USB 总线号**: `Bus 007`
- **USB 设备号**: `Device 003`
- **VID:PID**: `0403:6014` (注意：这是 **FT232H** 高速芯片，非普通的 FT232R)
- **内核驱动**: `ftdi_sio` (通过 `usbserial` 加载)
- **设备节点**: `/dev/ttyUSB0`
- **Udev 规则**: 已配置永久权限为 `0666` (规则文件: `/etc/udev/rules.d/99-u2d2.rules`)

## 3. 电机硬件参数 (Dynamixel RX-64)
- **型号**: RX-64
- **通信接口**: 4针 RS485 (使用类似 U2D2-PH 的转换器)
- **供电要求**: **12V ~ 18V 外部供电** (严禁使用 USB 5V 供电，会烧毁接口)
- **接线定义 (4线制)**:
  - `GND` <-> 舵机 `GND` (黑线)
  - `VDD` <-> 舵机 `VDD` (红线，接外部电源)
  - `D+`  <-> 舵机 `D+`  (数据+)
  - `D-`  <-> 舵机 `D-`  (数据-)

## 4. 通信协议参数 (关键配置)
在编写任何 `dynamixel_sdk` 或 ROS2 驱动时，**必须**严格使用以下参数，否则通信失败：

| 参数 | 值 | 备注 |
| :--- | :--- | :--- |
| `DEVICENAME` | `/dev/ttyUSB0` | 固定节点 |
| `PROTOCOL_VERSION` | `1.0` | **非 2.0**，老款 RX 系列专用 |
| `BAUDRATE` | `57600` | 已实测确认，非默认的 1Mbps |
| `SERVO_ID` | `6` | **非出厂默认的 1**，已被前期改过 |
| `SDK库名` | `dynamixel_sdk` | Python 包名 |

## 5. 寄存器地址映射表 (Protocol 1.0, 2 Bytes)
给电机发送指令或读取状态时，需使用以下地址和数据长度：

| 功能 | 地址 | 数据长度 | 取值范围 | 说明 |
| :--- | :--- | :--- | :--- | :--- |
| 扭矩开关 | `24` | 1 Byte | `0`(关), `1`(开) | 运动前必须设为 1 |
| 目标位置 | `30` | 2 Bytes | `0 ~ 1023` | `0`=0°, `512`=150°, `1023`=300° |
| 运动速度 | `32` | 2 Bytes | `0 ~ 1023` | 0 为最大速度，其他值越小越慢 |
| 当前位置 | `36` | 2 Bytes | `0 ~ 1023` | 只读反馈 |
| 当前速度 | `38` | 2 Bytes | `0 ~ 1023` | 只读反馈 |
| 当前电压 | `42` | 1 Byte | - | 实际电压 = 读取值 × 0.1 (V) |
| 当前温度 | `43` | 1 Byte | - | 单位: °C |

## 6. Python SDK 最小可运行模板
当使用 Codex 生成新代码时，请基于此模板扩展：

```python
from dynamixel_sdk import PacketHandler, PortHandler, COMM_SUCCESS

DEVICENAME = "/dev/ttyUSB0"
BAUDRATE = 57600
PROTOCOL_VERSION = 1.0
SERVO_ID = 6

ADDR_TORQUE_ENABLE = 24
ADDR_GOAL_POSITION = 30
ADDR_PRESENT_POSITION = 36

port = PortHandler(DEVICENAME)
packet = PacketHandler(PROTOCOL_VERSION)

if not port.openPort():
    raise RuntimeError(f"无法打开串口: {DEVICENAME}")

if not port.setBaudRate(BAUDRATE):
    port.closePort()
    raise RuntimeError(f"无法设置波特率: {BAUDRATE}")

model_num, result, error = packet.ping(port, SERVO_ID)
if result != COMM_SUCCESS:
    raise RuntimeError(packet.getTxRxResult(result))
if error != 0:
    raise RuntimeError(packet.getRxPacketError(error))

print(f"找到舵机: ID={SERVO_ID}, Model=0x{model_num:04X}")

result, error = packet.write1ByteTxRx(port, SERVO_ID, ADDR_TORQUE_ENABLE, 1)
if result != COMM_SUCCESS:
    raise RuntimeError(packet.getTxRxResult(result))
if error != 0:
    raise RuntimeError(packet.getRxPacketError(error))

goal_position = 600
result, error = packet.write2ByteTxRx(port, SERVO_ID, ADDR_GOAL_POSITION, goal_position)
if result != COMM_SUCCESS:
    raise RuntimeError(packet.getTxRxResult(result))
if error != 0:
    raise RuntimeError(packet.getRxPacketError(error))

present_position, result, error = packet.read2ByteTxRx(port, SERVO_ID, ADDR_PRESENT_POSITION)
if result != COMM_SUCCESS:
    raise RuntimeError(packet.getTxRxResult(result))
if error != 0:
    raise RuntimeError(packet.getRxPacketError(error))

print(f"当前位置: {present_position}")
port.closePort()
```

## 7. ROS2 落地建议

你已经确认了底层串口通信，下一步不要再纠结扫描程序，而是直接进入 ROS2 封装。仓库里新增的
[ros2_ws/src/aloha_rx64_bridge](/home/dlts/Openpi/ros2_ws/src/aloha_rx64_bridge)
就是基于本页参数写的最小 ROS2 桥接包，默认配置如下：

- `device_name=/dev/ttyUSB0`
- `baudrate=57600`
- `servo_id=6`
- `protocol=1.0`

建议流程：

1. 先用 `rx64_ping` 做启动前自检。
2. 再用 `rx64_single_node` 发布 `/joint_states` 并接收控制命令。
3. 单舵机确认稳定后，再扩展到 ALOHA 多关节映射。

如果 Orange Pi 上出现“`dynamixel_sdk` 脚本能动，但 `ros2 run` / `colcon build` 不工作”，优先改用直接脚本方式：

- [scripts/rx64_ros2_node.py](/home/dlts/Openpi/scripts/rx64_ros2_node.py)

推荐命令：

```bash
source /opt/ros/foxy/setup.bash
/usr/bin/python3 ~/Openpi/scripts/rx64_ros2_node.py
```

控制命令：

```bash
ros2 topic pub --once /rx64/goal_position_deg std_msgs/msg/Float64 "{data: 10.0}"
ros2 topic pub --once /rx64/goal_position_raw std_msgs/msg/UInt16 "{data: 512}"
ros2 service call /rx64/torque_enable std_srvs/srv/SetBool "{data: true}"
ros2 topic echo /joint_states
```
