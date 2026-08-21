Author: rxy
Last Update: 2026/08/13

## ⚠️ 重要更新 (2026-08-13)

### 电机控制系统升级

系统已升级为 **多圈位置闭环控制命令2 (0xA4)**，支持位置+速度联合控制。

- 📄 详细说明: [MOTOR_CONTROL_UPGRADE.md](MOTOR_CONTROL_UPGRADE.md)
- 🔍 接口检查: [INTERFACE_CHECK_REPORT.md](INTERFACE_CHECK_REPORT.md)
- 🚀 快速参考: [QUICK_REFERENCE.md](QUICK_REFERENCE.md)
- 🧪 测试脚本: `./test_motor_control.sh`

### Gripper接口升级

Gripper舵机接口已从原始编码值升级为 **角度（度）** 单位。

---

## 🤖 机械臂控制指令

### 1. 末端位姿控制

#### 话题: `/target_ee_pose`
**消息类型**: `std_msgs/msg/Float64MultiArray`

**数据格式**:
```yaml
data: [x, y, z, alpha, theta, gripper_angle]
```

| 参数 | 单位 | 说明 |
|------|------|------|
| x | mm | 末端X坐标 |
| y | mm | 末端Y坐标 |
| z | mm | 末端Z坐标 |
| alpha | deg | 末端俯仰角 |
| theta | deg | 腕部旋转角 |
| gripper_angle | deg | 夹爪开合角度 |

**示例**:
```bash
# 移动到位置 (300mm, 0mm, 200mm)，俯仰45°，夹爪开合30°
ros2 topic pub --once /target_ee_pose std_msgs/msg/Float64MultiArray \
  "{data: [300.0, 0.0, 200.0, 45.0, 0.0, 30.0]}"
```

### 2. 关节角度控制

#### 话题: `/manip/joint_cmd`
**消息类型**: `sensor_msgs/msg/JointState`

**数据格式**:
```yaml
name: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
position: [j1, j2, j3, j4, j5, j6]  # 单位: rad
velocity: [v1, v2, v3, v4, v5, v6]  # 单位: rad/s
```

**示例**:
```bash
# 所有关节回到零点
ros2 topic pub --once /manip/joint_cmd sensor_msgs/msg/JointState \
  "{name: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'], \
    position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], \
    velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

### 3. Gripper控制

#### 话题: `/gripper/joint_cmd`
**消息类型**: `sensor_msgs/msg/JointState`

**数据格式**:
```yaml
name: [joint_4, joint_5, joint_6]
position: [angle4, angle5, angle6]  # 单位: degree (度)
velocity: [speed4, speed5, speed6]  # 舵机原始速度值 (0~3400, 无量纲)
effort: [acc4, acc5, acc6]          # 舵机原始加速度值 (0~254, 无量纲)
```

**参数说明**:
- `position`: 目标角度（度），相对于零点（编码值2048）
- `velocity`: 舵机速度编码，范围 0~3400
  - 值越大，舵机转动越快
  - **不是** deg/s，是舵机内部编码值
  - 默认值: 1000（当输入为0时使用）
- `effort`: 舵机加速度编码，范围 0~254
  - 控制启停时的加速度
  - **不是** deg/s²，是舵机内部编码值
  - 默认值: 50（当输入为0时使用）

**示例**:
```bash
# 控制gripper所有舵机到90°，速度1000，加速度50
ros2 topic pub --once /gripper/joint_cmd sensor_msgs/msg/JointState \
  "{name: ['joint_4', 'joint_5', 'joint_6'], \
    position: [90.0, 90.0, 90.0], \
    velocity: [1000, 1000, 1000], \
    effort: [50, 50, 50]}"

# 快速运动：速度3000，加速度100
ros2 topic pub --once /gripper/joint_cmd sensor_msgs/msg/JointState \
  "{name: ['joint_4', 'joint_5', 'joint_6'], \
    position: [45.0, 45.0, 45.0], \
    velocity: [3000, 3000, 3000], \
    effort: [100, 100, 100]}"

# 慢速运动：速度500，加速度20
ros2 topic pub --once /gripper/joint_cmd sensor_msgs/msg/JointState \
  "{name: ['joint_4', 'joint_5', 'joint_6'], \
    position: [-30.0, -30.0, -30.0], \
    velocity: [500, 500, 500], \
    effort: [20, 20, 20]}"

# 控制单个舵机
ros2 topic pub --once /gripper/joint_cmd sensor_msgs/msg/JointState \
  "{name: ['joint_5'], \
    position: [45.0], \
    velocity: [1000], \
    effort: [50]}"
```

### 4. 系统使能控制

#### 话题: `/arm/state_cmd`
**消息类型**: `std_msgs/msg/UInt8MultiArray`

**数据格式**:
```yaml
data: [motor_id, enable_flag]
```

| 参数 | 值 | 说明 |
|------|-----|------|
| motor_id | 0 | 所有电机 |
| motor_id | 1-6 | 指定电机ID |
| enable_flag | 0 | 失能 |
| enable_flag | 1 | 使能 |

**示例**:
```bash
# 使能所有电机
ros2 topic pub --once /arm/state_cmd std_msgs/msg/UInt8MultiArray "{data: [0, 1]}"

# 失能所有电机
ros2 topic pub --once /arm/state_cmd std_msgs/msg/UInt8MultiArray "{data: [0, 0]}"

# 使能电机1
ros2 topic pub --once /arm/state_cmd std_msgs/msg/UInt8MultiArray "{data: [1, 1]}"
```

---

## 📊 状态监控

### 1. 机械臂关节状态

#### 话题: `/manip/joint_states`
**消息类型**: `sensor_msgs/msg/JointState`

**数据格式**:
```yaml
name: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
position: [j1, j2, j3, j4, j5, j6]  # 单位: rad
velocity: [v1, v2, v3, v4, v5, v6]  # 单位: rad/s (当前为空)
```

**监控命令**:
```bash
# 实时查看
ros2 topic echo /manip/joint_states

# 查看一次
ros2 topic echo /manip/joint_states --once

# 查看频率
ros2 topic hz /manip/joint_states
```

### 2. Gripper状态

#### 话题: `/gripper/joint_states`
**消息类型**: `sensor_msgs/msg/JointState`

**数据格式**:
```yaml
name: [servo_4, servo_5, servo_6]
position: [angle4, angle5, angle6]  # 单位: degree (度)
velocity: []  # 暂未实现
```

**监控命令**:
```bash
# 实时查看
ros2 topic echo /gripper/joint_states

# 查看一次
ros2 topic echo /gripper/joint_states --once
```

### 3. 电机原始状态

#### 话题: `/arm/motor_states`
**消息类型**: `std_msgs/msg/Float32MultiArray`

**数据格式**:
```yaml
data: [motor_angle_1, motor_angle_2, motor_angle_3, ...]  # 单位: degree
```

**监控命令**:
```bash
# 实时查看电机角度
ros2 topic echo /arm/motor_states
```

### 4. CAN总线监控

```bash
# 监控所有CAN消息
candump can0

# 过滤特定ID (例如电机1, ID=0x141)
candump can0 | grep 141

# 过滤命令类型 (例如0xA4命令)
candump can0 | grep "A4"

# 查看CAN总线统计
canbusload can0@1000000 -r
```

### 5. 系统诊断

```bash
# 查看所有话题
ros2 topic list

# 查看话题频率
ros2 topic hz /manip/joint_states
ros2 topic hz /gripper/joint_states

# 查看话题延迟
ros2 topic delay /manip/joint_cmd

# 查看节点信息
ros2 node info /motor_controller_node
ros2 node info /servo_controller_node
```

---

## 🚀 快速启动

### 设置波特率并开启接口
```bash
sudo ip link set can0 up type can bitrate 1000000
```

### MotorController节点运行
```bash
ros2 launch rflymanip_bridge motor_controller.launch.py
```

### 节点运行
#### xml启动
```bash
ros2 launch ros2_socketcan socket_can_bridge.launch.xml \ receiver_interval_sec:=0.001 \ sender_timeout_sec:=0.001
```
#### ros2 run启动
##### 1.启动节点
```bash
ros2 run ros2_socketcan socket_can_sender_node_exe --ros-args -p interface:=can0
ros2 run ros2_socketcan socket_can_receiver_node_exe --ros-args -p interface:=can0
```
##### 2.配置节点
```bash
ros2 lifecycle set /socket_can_sender_node configure
ros2 lifecycle set /socket_can_receiver_node configure
```
##### 3.激活节点
```bash
ros2 lifecycle set /socket_can_sender_node activate
ros2 lifecycle set /socket_can_receiver_node activate
```

### 自启动
#### 1.带硬件初始化
```bash
./devinit_can.sh
```

#### 2.仅启动ROS2节点
```bash
./start_bridge.sh
```

### 测试接收

```bash
ros2 topic pub --once /can0/rx can_msgs/msg/Frame "{id: 1, dlc: 8, data: [0xA1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]}"
```
