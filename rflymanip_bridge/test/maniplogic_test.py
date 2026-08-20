#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class ManipSequentialTestNode(Node):
    def __init__(self):
        super().__init__('manip_sequential_test_node')
        
        # 发布到 /manip_cmd 话题
        self.manip_pub = self.create_publisher(JointState, 'manip_cmd', 10)
        
        # 定时器频率 10Hz (0.1s) 用于刷新指令
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.joint_index = 0
        self.get_logger().info('Manip Sequential Test Node Started')
        self.get_logger().info('Each joint will move for 1 second in sequence.')

    def timer_callback(self):
        now = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - self.start_time
        
        # 每个关节总周期 4s (2s 在位置 A, 2s 在位置 B)
        cycle_duration = 4.0
        joint_cycle_index = int(elapsed // cycle_duration)
        self.joint_index = joint_cycle_index % 6
        
        # 确定在当前关节周期内是在位置 A 还是位置 B
        # sub_elapsed 0-2s 为位置 A, 2-4s 为位置 B
        sub_elapsed = elapsed % cycle_duration
        is_pos_b = sub_elapsed > (cycle_duration / 2.0)
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['motor_1', 'motor_2', 'motor_3', 'joint_4', 'joint_5', 'joint_6']
        
        # 基础位置
        base_positions = [0.0, 1.57, -1.57, 0.0, 0.0, 0.0]
        
        # 运动幅度: 关节 2 (index 1) 调小，其他保持 0.4
        current_amp = 0.2 if self.joint_index == 1 else 0.4
        offset = current_amp if is_pos_b else -current_amp
        
        # 构造当前位置数组
        positions = list(base_positions)
        positions[self.joint_index] += offset
        
        msg.position = positions
        msg.velocity = [0.0] * 6
        msg.effort = [0.0] * 6
        
        self.manip_pub.publish(msg)
        
        # 每秒打印一次状态切换
        state_label = "Pos B" if is_pos_b else "Pos A"
        if int(elapsed * 10) % 10 == 0:
            self.get_logger().info(f'Moving Joint: {msg.name[self.joint_index]} | State: {state_label}')

def main(args=None):
    rclpy.init(args=args)
    node = ManipSequentialTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Test stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
