# RFly Arm

ROS2 机械臂控制系统，支持 Docker 容器化部署。

**项目历史**：本代码库由 rxy 最初创建，随后 zzx 和 wt 基于原版进行改编，完成与 IsaacSim 仿真环境的适配，后续部署于地面六自由度平台进行实际测试验证。

## 系统要求

- Ubuntu 22.04+
- ROS2 Humble
- Docker & Docker Compose
- 网络模式：`host`

---

## ⚡ 快速开始

### 1. 下载项目

```bash
# 安装 Git LFS（用于下载 Docker 镜像）
sudo apt-get install git-lfs

# 克隆项目
git clone https://github.com/xuan-shen-a/rflyarm.git
cd rflyarm
```

### 2. 启动系统

```bash
# 一键启动（推荐）
source ./docker-start.sh
# 提示"是否需要编译代码？" → 按需选择 y/N
# 菜单选择 1 - 启动服务+当前终端配置ROS2+新终端显示日志

# 发送末端位姿命令
ros2 topic pub --once /target_ee_pose std_msgs/msg/Float64MultiArray \
  "{data: [400.0, 0.0, 400.0, 0.0, 0.0, 0.5]}"

# 查看状态（建议进入control-test容器查看）
ros2 topic echo /arm/joint_states
```

---

## 🤖 控制指令

### 0. 话题架构说明

系统使用分层控制架构：

```
/manip_cmd (6个关节)
    ↓ manip_logic_node 分发
    ├─→ /arm/joint_cmd (关节1、2、3 - 电机)
    └─→ /gripper/joint_cmd (关节4、5、6 - 舵机)
```

- **统一控制**：发送 `/manip_cmd` 可同时控制所有6个关节
- **分离控制**：直接发送 `/arm/joint_cmd` 或 `/gripper/joint_cmd` 可单独控制电机或舵机

### 1. 末端位姿控制（推荐）

**话题**: `/target_ee_pose`  
**格式**: `[x, y, z, alpha, theta, gripper_angle]`

| 参数 | 单位 | 说明 |
|------|------|------|
| x, y, z | mm | 末端位置坐标 |
| alpha | rad | 末端俯仰角 |
| theta | rad | 腕部旋转角 |
| gripper_angle | rad | 夹爪开合角度 |

```bash
# 示例：移动到 (400mm, 0mm, 400mm)，俯仰0°
ros2 topic pub --once /target_ee_pose std_msgs/msg/Float64MultiArray \
  "{data: [400.0, 0.0, 400.0, 0.0, 0.0, 0.5]}"
```

### 2. 六关节统一控制

**话题**: `/manip_cmd`  
**格式**: `name[6]` (必需), `position[6]` (必需, 单位: rad), `velocity[6]` (可选, rad/s), `effort[6]` (可选, 仅对关节4~6有效)

**重要：`/manip_cmd` 使用 IK 逻辑关节坐标系，与电机硬件零位不同。**  
`manip_logic_node` 会对关节1~3施加偏置和方向变换后再发给电机：

| 关节 | 偏置 `joint_offsets_` | 方向 | 说明 |
|------|----------------------|------|------|
| joint_1 | 0 | +1 | 与电机坐标系相同 |
| joint_2 | π/2 | +1 | 逻辑0对应电机 -π/2 |
| joint_3 | 0 | -1 | 含联动补偿 `in2+in1-π` |
| joint_4~6 | 0 | 各自配置 | 由舵机节点处理方向 |

因此 **`position=[0,0,0,0,0,0]` 不是机械臂所有关节的硬件零位**，而是 IK 定义的初始逻辑姿态。若需直接按硬件角度控制电机，请使用 `/arm/joint_cmd`（见第3节）。

**字段说明**:

| 字段 | 必需 | 说明 |
|------|------|------|
| `name` | ✅ 必需 | 必须为 6 个关节名 |
| `position` | ✅ 必需 | IK 逻辑关节角度，rad，需恰好 6 个 |
| `velocity` | 可选 | 缺失或为0时下游节点使用各自默认值 |
| `effort` | 可选 | 关节1-3（电机）：当前多圈位置控制模式2下忽略；关节4-6（舵机）：加速度编码 0-254，缺失时舵机节点套用默认值 50 |

```bash
# 通常由 manip_controller_node 自动发布，不建议手动发送
# 如确需手动发送，position 填入的是 IK 逻辑坐标，不是硬件角度
ros2 topic pub --once /manip_cmd sensor_msgs/msg/JointState \
  "{name: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'], \
    position: [0.0, 1.5708, 1.5708, 0.0, 0.0, 0.0], \
    velocity: [0.2, 0.2, 0.2, 1.2, 1.2, 1.2], \
    effort: [0.0, 0.0, 0.0, 40.0, 40.0, 40.0]}"
```

**说明**：
- 发送到 `/manip_cmd` 的命令会被 `manip_logic_node` 自动分发：
  - 关节1、2、3 → `/arm/joint_cmd` (电机，经偏置/方向变换)
  - 关节4、5、6 → `/gripper/joint_cmd` (舵机)
- 如需直接控制到硬件零位，请用 `/arm/joint_cmd` 和 `/gripper/joint_cmd` 分别发送

### 3. 电机关节控制（关节1、2、3）

**话题**: `/arm/joint_cmd`  
**格式**: `position[3]` (单位: rad), `velocity[3]` (单位: rad/s)

```bash
# 仅控制电机关节（1、2、3）
ros2 topic pub --once /arm/joint_cmd sensor_msgs/msg/JointState \
  "{name: ['joint_1', 'joint_2', 'joint_3'], \
    position: [0.0, 0.0, 0.0], \
    velocity: [0.2, 0.2, 0.2]}"
```

**说明**：
- 位置+速度联合控制（电机控制命令2，0xA4）
- `velocity` 为关节速度限制（rad/s），电机会在此速度范围内运动
- 省略 `velocity` 时默认为0，电机将使用内部默认速度（会产生警告）

### 4. Gripper控制（关节4、5、6）

**话题**: `/gripper/joint_cmd`  
**格式**: `position[3]` (单位: **rad**), `velocity[3]` (单位: **rad/s**), `effort[3]` (舵机加速度编码)

**参数说明**:
- `position`: 目标角度（**弧度 rad**）
- `velocity`: 目标角速度（**rad/s**），内部自动转换为舵机编码值（0~3400）
  - 转换关系: 1 编码单位 = 0.0015339808 rad/s
  - 例如: 1.5 rad/s ≈ 编码值 977
- `effort`: 加速度编码（0~254，无量纲，控制启停加速度）

```bash
# 控制所有舵机到0弧度，速度 1.2 rad/s, 加速度编码 40
ros2 topic pub --once /gripper/joint_cmd sensor_msgs/msg/JointState \
  "{name: ['joint_4', 'joint_5', 'joint_6'], \
    position: [0.0, 0.0, 0.0], \
    velocity: [1.2, 1.2, 1.2], \
    effort: [40.0, 40.0, 40.0]}"
```

**关节限位**:
- joint_4: -π/2 ~ π/2 (-90° ~ 90°)
- joint_5: -π/2 ~ π/2 (-90° ~ 90°)
- joint_6: 0 ~ π/3 (0° ~ 60°)

### 5. 系统使能

**电机使能/失能**:

**话题**: `/arm/state_cmd`  
**格式**: `[motor_id, enable_flag]`

```bash
# 使能所有电机
ros2 topic pub --once /arm/state_cmd std_msgs/msg/UInt8MultiArray "{data: [0, 1]}"

# 失能所有电机
ros2 topic pub --once /arm/state_cmd std_msgs/msg/UInt8MultiArray "{data: [0, 0]}"

# 设置电机零点（写入ROM）
ros2 topic pub --once /arm/state_cmd std_msgs/msg/UInt8MultiArray "{data: [1, 2]}"
```

**舵机使能/失能** (关节4、关节5、关节6):

舵机的使能状态由 `servo_controller_node` 节点自动管理：
- **使能**：节点启动时自动使能所有舵机（ID: 4, 5, 6）
- **失能**：节点关闭时自动失能所有舵机

```bash
# 舵机节点会在容器启动时自动运行
# 如需手动控制节点：

# 查看舵机节点状态
ros2 node list | grep servo

# 重启舵机节点（在容器内）
docker exec -it rflymanip_motor bash -c "ros2 run rflymanip_bridge servo_controller_node.py"
```

---

## 📊 状态监控

```bash
# 机械臂关节状态
ros2 topic echo /arm/joint_states

# Gripper状态（弧度 rad）
ros2 topic echo /gripper/joint_states

# 五次多项式轨迹位置、速度
ros2 topic echo /trajectory_debug

# 查看频率
ros2 topic hz /arm/joint_states  # 应为 ~20Hz

# CAN总线监控（容器内）
docker exec -it rflymanip_motor candump can0
```

---

## 🔀 系统数据流

```
                    ┌─────────────────────────────────────────────────────────────┐
                    │       上层：轨迹规划 (rflymanip_control)                     │
                    │                                                             │
                    │  manip_controller_node.cpp                                  │
                    │  velocity = sample.dq  (rad/s，五次多项式导数)               │
                    └───────────────────┬─────────────────────────────────────────┘
                                        │ /manip/joint_cmd
                                        │ position[6]: rad
                                        │ velocity[6]: rad/s  ← 全程 rad/s
                                        ▼
                    ┌─────────────────────────────────────────────────────────────┐
                    │       中层：逻辑分发 (rflymanip_bridge)                      │
                    │                                                             │
                    │  manip_logic_node.cpp                                       │
                    │  - 关节限位裁剪、偏置/方向变换                               │
                    │  - velocity 直接转发，不做单位转换                           │
                    └──────────┬────────────────────────────┬─────────────────────┘
                               │ /arm/joint_cmd             │ /gripper/joint_cmd
                               │ position[3]: rad           │ position[3]: rad
                               │ velocity[3]: rad/s         │ velocity[3]: rad/s ★改前为编码值
                               ▼                            ▼
          ┌─────────────────────────┐        ┌──────────────────────────────────────┐
          │  电机控制节点            │        │  舵机控制节点  ★ 本次修改             │
          │  motor_controller_node  │        │  servo_controller_node.py            │
          │  (关节 1、2、3)         │        │  (关节 4、5、6)                      │
          │  velocity: rad/s        │        │                                      │
          │  → CAN 电机驱动器       │        │  ★ 新增速度单位转换:                 │
          └─────────────────────────┘        │    speed_encode =                    │
                                             │    velocity_rad_s × (1/0.0015339808) │
                                             │    clamp(speed_encode, 0, 3400)      │
                                             └────────────────┬─────────────────────┘
                                                              │ 串口 /dev/ttyACM0
                                                              │ acc 编码 (0~254)
                                                              │ pos 编码 (0~4096)
                                                              │ speed 编码 (0~3400)  ← 转换后
                                                              ▼
                                             ┌──────────────────────────────────────┐
                                             │  hx_30_hm.py (servo_sdk)             │
                                             │  syncWritePosEx / writePosEx          │
                                             │  max(0, min(speed, 3400))             │
                                             └────────────────┬─────────────────────┘
                                                              │ 总线舵机协议
                                                              ▼
                                             ┌──────────────────────────────────────┐
                                             │  HX-30HM 舵机硬件                    │
                                             │  joint_4 / joint_5 / joint_6         │
                                             └──────────────────────────────────────┘
```

---

## 📂 项目结构

```
rflyarm/
├── docker-start.sh                      # 一键启动：编译 + 容器 + ROS2通信配置
├── docker-compose.yml                   # 服务定义：rflymanip_motor / rflymanip_control
├── setup_host_ros2.sh                   # 宿主机ROS2环境（UDP传输）
├── can.pdf                              # 电机CAN协议手册
├── config/
│   └── fastdds_udp_only.xml             # FastDDS仅UDP配置
├── rflymanip_bridge/                    # 硬件桥接层：电机 + 舵机驱动
│   ├── README.md                        # 详细接口文档 ⭐
│   ├── devinit_can.sh                   # CAN接口初始化（1 Mbit/s）
│   ├── start_bridge.sh                  # 容器内编译 + SocketCAN + 节点启动
│   ├── scripts/                         # 夹爪监控与转换测试脚本
│   ├── test/                            # 驱动、联动逻辑测试
│   └── src/rflymanip_bridge/
│       ├── src/                         # lkmotor_driver, motor_controller_node, manip_logic_node
│       ├── include/rflymanip_bridge/     # 对应头文件
│       ├── launch/                      # motor_controller.launch.py
│       ├── servo_controller_node.py     # 舵机控制节点
│       └── servo_sdk/                   # HX-30-HM 舵机协议SDK
├── rflymanip_control/                   # 运动学求解与轨迹规划
│   ├── README.md
│   └── src/rflymanip_control/
│       ├── src/                         # ik_node, ik_solver, manip_controller_node,
│       │                                # quintic_trajectory, path_planning, road.py
│       └── include/rflymanip_control/    # 对应头文件
└── rflymanip_demo/                      # 演示用OCI镜像归档（Git LFS）
```

> `build/`、`install/`、`log/` 为编译产物，已在 `.gitignore` 中排除。

---

## License

[添加你的许可证信息]
