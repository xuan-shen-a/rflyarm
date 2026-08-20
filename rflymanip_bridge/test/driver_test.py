#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt8MultiArray
import math
import time

class DriverTestNode(Node):
    def __init__(self):
        super().__init__('driver_test_node')
        
        # Publishers for Joint Commands
        self.arm_pub = self.create_publisher(JointState, '/arm/joint_cmd', 10)
        self.gripper_pub = self.create_publisher(JointState, '/gripper/joint_cmd', 10)
        
        # Publisher for System State (to enable the arm motors)
        self.state_pub = self.create_publisher(UInt8MultiArray, '/arm/state_cmd', 10)
        
        # Timer for joint commands (50Hz)
        self.timer = self.create_timer(0.02, self.timer_callback)
        
        # Initial status: Enable the arm system
        self.arm_enabled = False
        self.get_logger().info('Driver Test Node Initialized. Attempting to enable arm system...')
        
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        
    def enable_arm_system(self):
        """Sends a message to /arm/state_cmd to enable the motor controller system."""
        msg = UInt8MultiArray()
        # [0, 1] means target_id=0 (system), state=1 (enabled)
        msg.data = [0, 1]
        self.state_pub.publish(msg)
        self.get_logger().info('Sent enable command to /arm/state_cmd')

    def timer_callback(self):
        # Periodically ensure arm is enabled for the test
        now = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - self.start_time
        
        if not self.arm_enabled or int(elapsed * 50) % 500 == 0: # Every 10 seconds
            self.enable_arm_system()
            self.arm_enabled = True

        # Calculate a smooth oscillating position (sine wave)
        # Period: 4 seconds, Amplitude: 0.5 radians (~28.6 degrees)
        target_pos = 0.5 * math.sin(2 * math.pi * elapsed / 5.0)
        
        # 1. Arm Command (/arm/joint_cmd)
        # motor_controller_node.cpp handles IDs based on name suffix
        # Default arm IDs: 1, 2, 3
        arm_msg = JointState()
        arm_msg.header.stamp = self.get_clock().now().to_msg()
        # Using IDs 1, 2, and 3
        arm_msg.name = ['motor_1', 'motor_2', 'motor_3']
        arm_msg.position = [target_pos, 0.314, target_pos]
        arm_msg.velocity = [0.0, 0.0, 0.0]
        arm_msg.effort = [0.0, 0.0, 0.0]
        self.arm_pub.publish(arm_msg)
        
        # 2. Gripper Command (/gripper/joint_cmd)
        # 位置: 弧度 rad, 速度: rad/s, 加速度: 编码值 (0~254)
        # servo_controller_node.py handles IDs based on name suffix
        # Default servo IDs: 4, 5, 6
        gripper_msg = JointState()
        gripper_msg.header.stamp = self.get_clock().now().to_msg()
        gripper_msg.name = ['joint_4', 'joint_5', 'joint_6']
        gripper_msg.position = [1.57, target_pos, target_pos]  # 弧度
        gripper_msg.velocity = [1.2, 1.2, 1.2]  # rad/s
        gripper_msg.effort = [40.0, 40.0, 40.0]  # 加速度编码
        self.gripper_pub.publish(gripper_msg)
        
        # Log feedback
        if int(elapsed * 50) % 50 == 0: # 1Hz logging
            self.get_logger().info(f'Polling Test | Elapsed: {elapsed:.1f}s | Target Pos: {target_pos:.3f} rad')

def main(args=None):
    rclpy.init(args=args)
    node = DriverTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Test stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
