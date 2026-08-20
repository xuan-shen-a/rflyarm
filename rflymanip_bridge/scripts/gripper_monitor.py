#!/usr/bin/env python3
"""
Gripper位置监控工具
实时显示舵机位置的原始值和转换后的角度
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class GripperMonitor(Node):
    def __init__(self):
        super().__init__('gripper_monitor')

        # 默认零点（可根据实际情况调整）
        self.zero_offsets = {
            4: 2048,  # servo_4 零点
            5: 2048,  # servo_5 零点
            6: 2048,  # servo_6 零点
        }

        self.subscription = self.create_subscription(
            JointState,
            '/gripper/joint_states',
            self.callback,
            10)

        self.get_logger().info('Gripper Monitor Started')
        self.get_logger().info(f'Zero offsets: {self.zero_offsets}')
        self.get_logger().info('Press Ctrl+C to exit')
        print("\n" + "="*80)

    def raw_to_deg(self, servo_id, raw_value):
        """原始值 → 角度(度)"""
        zero = self.zero_offsets.get(servo_id, 2048)
        return (raw_value - zero) * 0.0879

    def raw_to_rad(self, servo_id, raw_value):
        """原始值 → 角度(弧度)"""
        zero = self.zero_offsets.get(servo_id, 2048)
        return (raw_value - zero) * 0.001534

    def callback(self, msg):
        print("\n" + "="*80)
        print(f"{'Servo':<12} | {'Raw Value':>10} | {'Angle (deg)':>12} | {'Angle (rad)':>12}")
        print("-"*80)

        for name, pos in zip(msg.name, msg.position):
            try:
                servo_id = int(name.split('_')[-1])
            except (ValueError, IndexError):
                continue

            deg = self.raw_to_deg(servo_id, pos)
            rad = self.raw_to_rad(servo_id, pos)

            print(f"{name:<12} | {pos:10.1f} | {deg:12.2f}° | {rad:12.4f} rad")

        print("="*80)


def main(args=None):
    rclpy.init(args=args)
    node = GripperMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nGripper Monitor Stopped")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
