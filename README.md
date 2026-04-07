# Orange Pi AI Pro 上用 RX-64 打通 ALOHA 的 ROS2 控制链路

这个仓库现在的目标不是一步到位替代 ALOHA 官方整套 Dynamixel 方案，而是先把你已经验证成功的 `RX-64 + U2D2 + /dev/ttyUSB0` 接入 ROS2，形成一条最小可用控制链路：

`ROS2 Topic / Service -> rclpy Node -> dynamixel_sdk -> U2D2(RS485) -> RX-64`

## 已确认的现场参数

这些参数来自现有 [code.md](/home/dlts/Openpi/code.md) 和你的实测日志，后面的代码默认就按这个配置写：

- 设备节点: `/dev/ttyUSB0`
- 舵机型号: `Dynamixel RX-64`
- 通信协议: `Protocol 1.0`
- 波特率: `57600`
- 舵机 ID: `6`
- 通信方式: `RS485 4Pin`
- 供电: 外部 `12V~18V`

你的扫描结果里最关键的是：

- `Protocol 1.0`
- `ID=6`
- `型号=0x0040`

`0x0040` 就是 `64`，对应 `RX-64`，说明硬件链路已经通了。

## 目录

- [code.md](/home/dlts/Openpi/code.md): 现场硬件与寄存器记录
- [ros2_ws/src/aloha_rx64_bridge](/home/dlts/Openpi/ros2_ws/src/aloha_rx64_bridge): 最小可用 ROS2 控制包

## Orange Pi 上的建议运行方式

最稳妥的方式是：

1. ROS2 节点运行在系统 ROS2 Python 环境里。
2. `lerobot` conda 环境保留给上层算法、策略或数据采集。
3. 如果你坚持在 `lerobot` 里跑 ROS2，也可以，但必须先验证：

```bash
python -c "import rclpy; import dynamixel_sdk"
```

如果这条命令失败，不要继续在 conda 环境里构建 ROS2 包，直接切回系统 ROS2 环境。

## 部署步骤

以下命令在 Orange Pi 上执行。

### 1. 准备 ROS2 环境

Ubuntu 20.04 常见的是 ROS2 Foxy：

```bash
source /opt/ros/foxy/setup.bash
```

如果你的板子上不是 Foxy，而是别的 ROS2 版本，把上面的发行版名替换掉。

### 2. 安装 Python 依赖

```bash
python3 -m pip install --user dynamixel-sdk
```

如果你已经在目标环境里装过，可以跳过。

### 3. 构建工作区

把这个仓库放到 Orange Pi 后：

```bash
cd ~/Openpi/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 4. 先做非 ROS2 连通性确认

```bash
ros2 run aloha_rx64_bridge rx64_ping --ros-args \
  -p device_name:=/dev/ttyUSB0 \
  -p baudrate:=57600 \
  -p servo_id:=6
```

预期会打印型号、当前位置、电压、温度。

### 5. 启动单舵机 ROS2 节点

```bash
ros2 launch aloha_rx64_bridge rx64_single.launch.py
```

默认参数已经是：

- `device_name=/dev/ttyUSB0`
- `baudrate=57600`
- `servo_id=6`
- `joint_name=rx64_joint`

### 6. 发控制命令

直接发弧度命令，零点默认是 `raw=512`：

```bash
ros2 topic pub --once /rx64_single/goal_position_rad std_msgs/msg/Float64 "{data: 0.3}"
```

也可以发角度命令：

```bash
ros2 topic pub --once /rx64_single/goal_position_deg std_msgs/msg/Float64 "{data: 15.0}"
```

或者发绝对原始位置：

```bash
ros2 topic pub --once /rx64_single/goal_raw std_msgs/msg/UInt16 "{data: 600}"
```

### 7. 看反馈

```bash
ros2 topic echo /joint_states
```

### 8. 开关扭矩

```bash
ros2 service call /rx64_single/torque_enable std_srvs/srv/SetBool "{data: true}"
ros2 service call /rx64_single/torque_enable std_srvs/srv/SetBool "{data: false}"
```

## 和 ALOHA 的关系

这套代码当前解决的是“底层单个 RX-64 舵机接入 ROS2”的问题，不是完整的 ALOHA 机械臂控制栈。它的作用是：

- 先确认 Orange Pi 上的 ROS2 能稳定控制老款 Protocol 1.0 舵机
- 先跑通 `joint_states` 发布
- 先接受标准 `trajectory_msgs/JointTrajectory` 指令
- 后续你可以按同样模式，把多个舵机扩成多关节节点

如果下一步要往 ALOHA 贴近，通常会继续做这几件事：

1. 把单舵机节点扩成多舵机总线管理节点。
2. 为每个关节建立 `joint_name <-> servo_id <-> zero_offset <-> direction` 映射。
3. 接入 `ros2_control` 或你的上层策略节点。
4. 再考虑把 RX-64 替换为更接近 ALOHA 原方案的电机。

## 安全建议

- 上电前先确认外部供电极性，`RX-64` 不能吃 USB 5V。
- 首次动作建议只发很小的角度，比如 `5~10` 度。
- 默认零点是 `raw=512`，但你的机构装配零位可能不是这里，正式上机械臂前要重新标定。
- 如果机械限位未知，先只用 `goal_raw` 在小范围试探，不要一上来打满行程。
