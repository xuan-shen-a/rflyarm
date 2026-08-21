#!/usr/bin/env python3
"""
主手机械臂状态发布节点
读取真实机械臂的关节状态，用于驱动仿真从手
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, Bool
import numpy as np
from collections import deque

class LeaderArmPublisher(Node):
    def __init__(self):
        super().__init__('leader_arm_publisher')

        # 订阅真实机械臂状态
        self.real_joint_sub = self.create_subscription(
            JointState,
            '/arm/joint_states',  # 真实机械臂话题
            self.real_joint_callback,
            10
        )

        # 发布主手状态（给从手订阅）
        self.leader_joint_pub = self.create_publisher(
            JointState,
            '/leader/joint_states',
            10
        )

        self.leader_ee_pub = self.create_publisher(
            PoseStamped,
            '/leader/ee_pose',
            10
        )

        # 示教模式控制
        self.teaching_mode_pub = self.create_publisher(
            Bool,
            '/arm/teaching_mode',
            10
        )

        # 状态
        self.current_joints = np.zeros(6)
        self.joint_velocity = deque(maxlen=5)  # 用于平滑

        # 启用示教模式
        self.enable_teaching_mode()

        self.get_logger().info('✅ Leader Arm Publisher started')
        self.get_logger().info('📍 Please manually move the real arm to demonstrate')

    def enable_teaching_mode(self):
        """启用真实机械臂的示教模式（需要硬件支持）"""
        msg = Bool()
        msg.data = True
        self.teaching_mode_pub.publish(msg)
        self.get_logger().info('🔓 Teaching mode enabled - arm is compliant')

    def real_joint_callback(self, msg: JointState):
        """接收真实机械臂关节状态并转发"""
        if len(msg.position) < 6:
            self.get_logger().warn(f'⚠️  Received {len(msg.position)} joints, expected 6')
            return

        # 更新当前状态
        self.current_joints = np.array(msg.position[:6])

        # 计算速度（用于判断是否在移动）
        if len(msg.velocity) >= 6:
            vel = np.linalg.norm(msg.velocity[:6])
            self.joint_velocity.append(vel)

        # 转发给从手
        leader_msg = JointState()
        leader_msg.header.stamp = self.get_clock().now().to_msg()
        leader_msg.header.frame_id = 'leader_base'
        leader_msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        leader_msg.position = self.current_joints.tolist()

        if len(msg.velocity) >= 6:
            leader_msg.velocity = list(msg.velocity[:6])

        self.leader_joint_pub.publish(leader_msg)

        # 计算并发布末端位姿（可选）
        ee_pose = self.compute_forward_kinematics(self.current_joints)
        self.publish_ee_pose(ee_pose)

    def compute_forward_kinematics(self, joints):
        """简单FK，返回末端位姿 [x, y, z, pitch, roll]"""
        # TODO: 实现你的机械臂FK
        # 这里返回占位符
        return np.array([0.5, 0.0, 0.5, 0.0, 0.0])

    def publish_ee_pose(self, ee_pose):
        """发布末端位姿"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'leader_base'
        msg.pose.position.x = float(ee_pose[0])
        msg.pose.position.y = float(ee_pose[1])
        msg.pose.position.z = float(ee_pose[2])
        # pitch, roll转四元数（简化）
        msg.pose.orientation.w = 1.0

        self.leader_ee_pub.publish(msg)

    def is_moving(self):
        """判断主手是否在移动"""
        if len(self.joint_velocity) == 0:
            return False
        avg_vel = np.mean(self.joint_velocity)
        return avg_vel > 0.01  # rad/s阈值

def main(args=None):
    rclpy.init(args=args)
    node = LeaderArmPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Stopping leader arm publisher')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
