# 真实机械臂模仿学习部署指南

## 1. 硬件准备清单

### 1.1 必需硬件
- [x] 六自由度机械臂（关节1-6）
- [x] 电机控制器（关节1-3，CAN总线）
- [x] 舵机控制器（关节4-6，串口/dev/ttyACM0）
- [ ] RGB-D 相机 × 2（手部 + 头部）
- [ ] 遥操作设备（可选）：
  - 3D SpaceMouse
  - VR 手柄（HTC Vive / Oculus）
  - 主机械臂（镜像控制）

### 1.2 相机安装位置
参考仿真配置 `RflyArm/simulation/world/camera_mount.py`：
- **手部相机**：安装于 `arm_link6`，俯视夹爪方向
- **头部相机**：固定于平台，光轴朝向工作区域

---

## 2. ROS2 数据采集节点实现

### 2.1 创建采集节点

创建文件：`rflyarm_bridge/scripts/teleoperation_recorder.py`

```python
#!/usr/bin/env python3
"""
真实机械臂遥操作数据采集节点
采集格式兼容 LeRobot v0.6.1
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image, CameraInfo
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import numpy as np
import h5py
from pathlib import Path
from datetime import datetime

class TeleoperationRecorder(Node):
    def __init__(self):
        super().__init__('teleoperation_recorder')
        
        # 订阅关节状态
        self.joint_sub = self.create_subscription(
            JointState, '/arm/joint_states', 
            self.joint_callback, 10
        )
        
        # 订阅相机（需要添加相机驱动）
        self.hand_camera_sub = self.create_subscription(
            Image, '/hand_camera/color/image_raw',
            self.hand_camera_callback, 10
        )
        self.head_camera_sub = self.create_subscription(
            Image, '/head_camera/color/image_raw',
            self.head_camera_callback, 10
        )
        
        # 订阅遥操作命令
        self.teleop_sub = self.create_subscription(
            Float64MultiArray, '/teleop_command',
            self.teleop_callback, 10
        )
        
        self.bridge = CvBridge()
        self.episode_data = []
        self.current_frame = {}
        self.recording = False
        
        self.get_logger().info('✅ Teleoperation Recorder initialized')
    
    def joint_callback(self, msg):
        """记录关节状态作为observation"""
        if self.recording:
            self.current_frame['observation_state'] = np.array(msg.position[:6])
            self.current_frame['timestamp'] = self.get_clock().now().nanoseconds / 1e9
    
    def hand_camera_callback(self, msg):
        """记录手部相机图像"""
        if self.recording:
            image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
            self.current_frame['hand_camera'] = image
    
    def head_camera_callback(self, msg):
        """记录头部相机图像"""
        if self.recording:
            image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
            self.current_frame['head_camera'] = image
    
    def teleop_callback(self, msg):
        """记录遥操作动作指令"""
        if self.recording:
            self.current_frame['action'] = np.array(msg.data[:6])
            
            # 当一帧数据完整时保存
            if self.is_frame_complete():
                self.episode_data.append(self.current_frame.copy())
                self.current_frame = {}
    
    def is_frame_complete(self):
        """检查一帧数据是否完整"""
        required_keys = ['observation_state', 'hand_camera', 
                        'head_camera', 'action', 'timestamp']
        return all(k in self.current_frame for k in required_keys)
    
    def start_recording(self):
        """开始记录episode"""
        self.recording = True
        self.episode_data = []
        self.get_logger().info('🔴 Recording started')
    
    def stop_recording(self, success=True):
        """停止记录并保存"""
        self.recording = False
        if not self.episode_data:
            self.get_logger().warn('⚠️  No data recorded')
            return
        
        # 保存为 LeRobot 格式
        output_dir = Path.home() / 'rflyarm' / 'collected_data'
        output_dir.mkdir(parents=True, exist_ok=True)
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = output_dir / f'episode_{timestamp}_{"success" if success else "fail"}.hdf5'
        
        self.save_episode(filename)
        self.get_logger().info(f'💾 Episode saved: {filename}')
    
    def save_episode(self, filename):
        """保存episode为HDF5格式"""
        with h5py.File(filename, 'w') as f:
            n_frames = len(self.episode_data)
            
            # 创建数据集
            f.create_dataset('observation/state', 
                           data=np.array([d['observation_state'] for d in self.episode_data]))
            f.create_dataset('action', 
                           data=np.array([d['action'] for d in self.episode_data]))
            f.create_dataset('timestamp', 
                           data=np.array([d['timestamp'] for d in self.episode_data]))
            
            # 保存图像
            hand_images = np.array([d['hand_camera'] for d in self.episode_data])
            head_images = np.array([d['head_camera'] for d in self.episode_data])
            f.create_dataset('observation/images/hand_camera', data=hand_images, compression='gzip')
            f.create_dataset('observation/images/head_camera', data=head_images, compression='gzip')
            
            f.attrs['fps'] = 50
            f.attrs['n_frames'] = n_frames

def main():
    rclpy.init()
    recorder = TeleoperationRecorder()
    
    # 可以通过服务或键盘控制开始/停止
    # 这里简化为一直录制
    recorder.start_recording()
    
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        recorder.stop_recording()
    
    recorder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2.2 遥操作接口

创建文件：`rflyarm_bridge/scripts/spacemouse_teleop.py`

```python
#!/usr/bin/env python3
"""
SpaceMouse 遥操作节点
将 3D 鼠标输入转换为机械臂命令
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import pyspacemouse

class SpaceMouseTeleop(Node):
    def __init__(self):
        super().__init__('spacemouse_teleop')
        
        self.cmd_pub = self.create_publisher(
            Float64MultiArray, '/teleop_command', 10
        )
        
        # 当前关节状态
        self.current_joints = np.zeros(6)
        
        # 打开 SpaceMouse
        success = pyspacemouse.open()
        if not success:
            self.get_logger().error('❌ Failed to open SpaceMouse')
            return
        
        self.get_logger().info('✅ SpaceMouse teleoperation active')
        
        # 定时读取输入
        self.timer = self.create_timer(0.02, self.teleop_loop)  # 50Hz
    
    def teleop_loop(self):
        """读取 SpaceMouse 并发布命令"""
        state = pyspacemouse.read()
        if state is None:
            return
        
        # 映射 6DOF 输入到关节增量
        # 这里需要根据你的运动学调整
        delta_joints = np.array([
            state.roll * 0.01,    # joint_1
            state.pitch * 0.01,   # joint_2
            state.yaw * 0.01,     # joint_3
            state.x * 0.005,      # joint_4
            state.y * 0.005,      # joint_5
            state.z * 0.005,      # joint_6 (gripper)
        ])
        
        self.current_joints += delta_joints
        
        # 发布命令
        msg = Float64MultiArray()
        msg.data = self.current_joints.tolist()
        self.cmd_pub.publish(msg)

def main():
    rclpy.init()
    node = SpaceMouseTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 3. 相机驱动集成

### 3.1 安装 RealSense 驱动（如果使用 Intel RealSense）

```bash
# 安装 librealsense
sudo apt-get install ros-humble-realsense2-camera

# 启动相机节点
ros2 run realsense2_camera realsense2_camera_node \
  --ros-args -p camera_name:=hand_camera
```

### 3.2 配置相机话题映射

确保相机发布到：
- `/hand_camera/color/image_raw`
- `/head_camera/color/image_raw`

---

## 4. 数据采集工作流

### 4.1 准备工作

```bash
# 1. 启动机械臂控制
source ./docker-start.sh

# 2. 启动相机
ros2 launch realsense2_camera rs_launch.py

# 3. 启动遥操作
python3 rflymanip_bridge/scripts/spacemouse_teleop.py

# 4. 启动数据记录
python3 rflymanip_bridge/scripts/teleoperation_recorder.py
```

### 4.2 采集流程

1. **初始化**：机械臂移动到安全起始位置
2. **开始记录**：按下 SpaceMouse 按钮或发送ROS服务
3. **执行任务**：通过遥操作完成灯泡拆卸
4. **停止记录**：任务完成后标记成功/失败
5. **重复**：采集 50-100 条成功轨迹

### 4.3 数据质量检查

```python
# 检查采集的数据
import h5py

with h5py.File('episode_20260820_120000_success.hdf5', 'r') as f:
    print(f"Frames: {f.attrs['n_frames']}")
    print(f"FPS: {f.attrs['fps']}")
    print(f"Observation shape: {f['observation/state'].shape}")
    print(f"Action shape: {f['action'].shape}")
```

---

## 5. 转换为 LeRobot 格式

创建转换脚本：`convert_to_lerobot.py`

```python
"""
将采集的 HDF5 数据转换为 LeRobot 数据集格式
"""
from pathlib import Path
import h5py
import numpy as np
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset

def convert_episodes_to_lerobot(
    input_dir: Path,
    output_dir: Path,
    dataset_name: str = "rflyarm_real_bulb_removal"
):
    """转换所有episode到LeRobot格式"""
    
    # 收集所有episode文件
    episode_files = sorted(input_dir.glob("episode_*_success.hdf5"))
    
    if not episode_files:
        print(f"❌ No episode files found in {input_dir}")
        return
    
    print(f"📂 Found {len(episode_files)} episodes")
    
    # 创建 LeRobot 数据集
    dataset = LeRobotDataset.create(
        repo_id=f"local/{dataset_name}",
        root=output_dir,
        fps=50,
        # 定义特征
        features={
            "observation.state": {"dtype": "float32", "shape": (6,)},
            "observation.images.hand_camera": {"dtype": "video"},
            "observation.images.head_camera": {"dtype": "video"},
            "action": {"dtype": "float32", "shape": (6,)},
        }
    )
    
    # 逐个添加episode
    for ep_idx, ep_file in enumerate(episode_files):
        print(f"Processing episode {ep_idx + 1}/{len(episode_files)}: {ep_file.name}")
        
        with h5py.File(ep_file, 'r') as f:
            n_frames = f.attrs['n_frames']
            
            for frame_idx in range(n_frames):
                frame = {
                    "observation.state": f['observation/state'][frame_idx],
                    "observation.images.hand_camera": f['observation/images/hand_camera'][frame_idx],
                    "observation.images.head_camera": f['observation/images/head_camera'][frame_idx],
                    "action": f['action'][frame_idx],
                    "episode_index": ep_idx,
                    "frame_index": frame_idx,
                    "timestamp": f['timestamp'][frame_idx],
                }
                dataset.add_frame(frame)
        
        dataset.consolidate_episode()
    
    print(f"✅ Dataset saved to {output_dir / dataset_name}")

if __name__ == '__main__':
    input_dir = Path.home() / 'rflyarm' / 'collected_data'
    output_dir = Path.home() / 'rflyarm' / 'lerobot_datasets'
    
    convert_episodes_to_lerobot(input_dir, output_dir)
```

---

## 6. 使用真实数据训练

### 6.1 纯真实数据训练

```bash
conda run -n lerobot lerobot-train \
  --dataset.repo_id=local/rflyarm_real_bulb_removal \
  --dataset.root="$HOME/rflyarm/lerobot_datasets/rflyarm_real_bulb_removal" \
  --policy.type=act \
  --output_dir="$HOME/rflyarm/outputs/train/act_real" \
  --wandb.enable=true \
  --wandb.project=rflyarm-real
```

### 6.2 仿真预训练 + 真实微调

```bash
# 第一步：仿真预训练（已完成）
# 使用 RflyArm/bulb_removal_il 的现有模型

# 第二步：真实数据微调
conda run -n lerobot lerobot-train \
  --dataset.repo_id=local/rflyarm_real_bulb_removal \
  --dataset.root="$HOME/rflyarm/lerobot_datasets/rflyarm_real_bulb_removal" \
  --policy.type=act \
  --pretrained_policy_path="$HOME/RflyArm/bulb_removal_il/outputs/train/act_bulb_remove_once/checkpoints/100000" \
  --output_dir="$HOME/rflyarm/outputs/train/act_finetuned" \
  --training.lr=1e-5 \  # 更小的学习率
  --training.num_epochs=50  # 更少的epoch
```

---

## 7. 真实机械臂部署

### 7.1 创建部署节点

创建文件：`rflyarm_bridge/scripts/act_real_deployment.py`

```python
#!/usr/bin/env python3
"""
在真实机械臂上部署 ACT 策略
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import numpy as np
import torch

# 导入 ACT 推理
import sys
sys.path.append(str(Path.home() / 'RflyArm' / 'bulb_removal_il' / 'src'))
from bulb_removal_il.inference.client import ACTInferenceClient

class ACTRealDeployment(Node):
    def __init__(self, checkpoint_path):
        super().__init__('act_real_deployment')
        
        # 加载 ACT 模型
        self.policy = self.load_policy(checkpoint_path)
        
        # 订阅观测
        self.joint_sub = self.create_subscription(
            JointState, '/arm/joint_states',
            self.joint_callback, 10
        )
        self.hand_cam_sub = self.create_subscription(
            Image, '/hand_camera/color/image_raw',
            self.hand_camera_callback, 10
        )
        self.head_cam_sub = self.create_subscription(
            Image, '/head_camera/color/image_raw',
            self.head_camera_callback, 10
        )
        
        # 发布控制指令
        self.cmd_pub = self.create_publisher(
            Float64MultiArray, '/target_ee_pose', 10
        )
        
        self.bridge = CvBridge()
        self.current_observation = {}
        
        # 控制循环
        self.timer = self.create_timer(0.02, self.control_loop)  # 50Hz
        
        self.get_logger().info('✅ ACT Real Deployment initialized')
    
    def load_policy(self, checkpoint_path):
        """加载 ACT 策略"""
        from lerobot.common.policies.act.modeling_act import ACTPolicy
        policy = ACTPolicy.from_pretrained(checkpoint_path)
        policy.eval()
        return policy
    
    def joint_callback(self, msg):
        self.current_observation['state'] = np.array(msg.position[:6])
    
    def hand_camera_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        self.current_observation['hand_camera'] = image
    
    def head_camera_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        self.current_observation['head_camera'] = image
    
    def control_loop(self):
        """主控制循环"""
        if not self.is_observation_complete():
            return
        
        # 推理动作
        action = self.infer_action(self.current_observation)
        
        # 安全检查
        action = self.apply_safety_limits(action)
        
        # 发布命令
        self.publish_action(action)
    
    def is_observation_complete(self):
        required = ['state', 'hand_camera', 'head_camera']
        return all(k in self.current_observation for k in required)
    
    def infer_action(self, observation):
        """使用 ACT 推理动作"""
        with torch.no_grad():
            obs_tensor = {
                'observation.state': torch.tensor(observation['state']).unsqueeze(0),
                'observation.images.hand_camera': torch.tensor(observation['hand_camera']).unsqueeze(0),
                'observation.images.head_camera': torch.tensor(observation['head_camera']).unsqueeze(0),
            }
            action = self.policy.select_action(obs_tensor)
        return action.cpu().numpy()[0]
    
    def apply_safety_limits(self, action):
        """应用安全限位"""
        # 关节限位
        joint_limits = np.array([
            [-np.pi, np.pi],      # joint_1
            [-np.pi/2, np.pi/2],  # joint_2
            [-np.pi/2, np.pi/2],  # joint_3
            [-np.pi/2, np.pi/2],  # joint_4
            [-np.pi/2, np.pi/2],  # joint_5
            [0, np.pi/3],         # joint_6
        ])
        return np.clip(action, joint_limits[:, 0], joint_limits[:, 1])
    
    def publish_action(self, action):
        """发布动作到真实机械臂"""
        msg = Float64MultiArray()
        msg.data = action.tolist()
        self.cmd_pub.publish(msg)

def main():
    rclpy.init()
    
    checkpoint = Path.home() / 'rflyarm' / 'outputs' / 'train' / 'act_finetuned' / 'checkpoints' / 'final'
    node = ACTRealDeployment(str(checkpoint))
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 7.2 部署流程

```bash
# 1. 启动机械臂控制系统
cd ~/rflyarm
source ./docker-start.sh

# 2. 启动相机
ros2 launch realsense2_camera rs_launch.py

# 3. 使能机械臂
ros2 topic pub --once /arm/state_cmd std_msgs/msg/UInt8MultiArray "{data: [0, 1]}"

# 4. 运行 ACT 策略
python3 rflymanip_bridge/scripts/act_real_deployment.py

# 5. 监控执行
ros2 topic echo /arm/joint_states
```

---

## 8. 调试与优化建议

### 8.1 常见问题

**问题1：动作抖动**
- 解决：增加动作平滑（移动平均、低通滤波）
- 减小学习率重新训练

**问题2：碰撞风险**
- 解决：添加虚拟力场（Virtual Fixtures）
- 实时碰撞检测

**问题3：泛化性差**
- 解决：增加数据多样性
- 使用域随机化

### 8.2 性能优化

```python
# 动作平滑
class ActionSmoother:
    def __init__(self, window_size=5):
        self.window = []
        self.window_size = window_size
    
    def smooth(self, action):
        self.window.append(action)
        if len(self.window) > self.window_size:
            self.window.pop(0)
        return np.mean(self.window, axis=0)
```

---

## 9. 评估指标

记录以下指标：
- **成功率**：完成任务的比例
- **执行时间**：完成任务的平均时间
- **平滑度**：动作加速度的标准差
- **安全性**：碰撞次数、超限次数

---

## 10. 下一步计划

- [ ] 安装并测试相机
- [ ] 实现遥操作接口
- [ ] 采集 20 条测试轨迹
- [ ] 转换为 LeRobot 格式
- [ ] 在仿真模型上微调
- [ ] 真实机械臂安全测试
- [ ] 大规模采集（100+ episodes）
- [ ] 完整训练与部署
