#!/usr/bin/env python3
"""
Gripper测试脚本
用于测试升级后的gripper接口（弧度 rad 和 rad/s 单位）
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class GripperTester(Node):
    def __init__(self):
        super().__init__('gripper_tester')

        # 订阅反馈
        self.state_sub = self.create_subscription(
            JointState, '/gripper/joint_states',
            self.state_callback, 10)

        # 发布命令
        self.cmd_pub = self.create_publisher(
            JointState, '/gripper/joint_cmd', 10)

        self.get_logger().info('='*60)
        self.get_logger().info('Gripper Tester Started')
        self.get_logger().info('Listening to /gripper/joint_states')
        self.get_logger().info('='*60)

    def state_callback(self, msg):
        """显示当前位置（弧度）"""
        print('\n' + '-'*60)
        print(f"{'Servo':<12} | {'Angle (rad)':>15} | {'Angle (deg)':>15}")
        print('-'*60)

        for name, pos in zip(msg.name, msg.position):
            angle_deg = pos * 180.0 / 3.14159265359
            print(f"{name:<12} | {pos:15.4f} rad | {angle_deg:15.2f}°")

        print('-'*60)

    def send_position(self, positions, speed=1.2, acc=40):
        """
        发送位置命令

        Args:
            positions: dict, {servo_id: angle_rad}
                      例如 {4: 0.785, 5: 1.57, 6: -0.523}  # 45°, 90°, -30°
            speed: float, 舵机角速度 (rad/s), 默认 1.5 rad/s
            acc: int, 舵机加速度编码 (0~254)
        """
        msg = JointState()

        for servo_id, angle_rad in positions.items():
            msg.name.append(f'joint_{servo_id}')
            msg.position.append(angle_rad)
            msg.velocity.append(speed)
            msg.effort.append(acc)

        self.cmd_pub.publish(msg)

        # 显示发送的命令
        print('\n' + '='*60)
        print('Sent command:')
        for servo_id, angle_rad in positions.items():
            angle_deg = angle_rad * 180.0 / 3.14159265359
            print(f'  joint_{servo_id}: {angle_rad:.4f} rad ({angle_deg:.2f}°) @ {speed:.2f} rad/s')
        print('='*60)


def main():
    rclpy.init()
    node = GripperTester()

    print("""
╔══════════════════════════════════════════════════════════════╗
║              Gripper Interface Test Menu                     ║
╚══════════════════════════════════════════════════════════════╝

Commands:
  1. Move to zero (0 rad)
  2. Move to +90° (1.57 rad)
  3. Move to -90° (-1.57 rad)
  4. Move to +45° (0.785 rad)
  5. Custom position
  q. Quit

Note: All positions are in radians, velocity in rad/s
""")

    import sys
    import select
    import termios
    import tty

    def get_key():
        """非阻塞读取键盘输入"""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    settings = termios.tcgetattr(sys.stdin)

    try:
        while rclpy.ok():
            # 处理ROS回调
            rclpy.spin_once(node, timeout_sec=0.1)

            # 检查键盘输入
            if select.select([sys.stdin], [], [], 0.0)[0]:
                key = get_key()

                if key == '1':
                    node.get_logger().info('Test 1: Move to zero (0 rad)')
                    node.send_position({4: 0.0, 5: 0.0, 6: 0.0}, speed=1.5)

                elif key == '2':
                    node.get_logger().info('Test 2: Move to +90° (1.57 rad)')
                    node.send_position({4: 1.5708, 5: 1.5708, 6: 1.5708}, speed=2.0)

                elif key == '3':
                    node.get_logger().info('Test 3: Move to -90° (-1.57 rad)')
                    node.send_position({4: -1.5708, 5: -1.5708, 6: -1.5708}, speed=2.0)

                elif key == '4':
                    node.get_logger().info('Test 4: Move to +45° (0.785 rad)')
                    node.send_position({4: 0.7854, 5: 0.7854, 6: 0.7854}, speed=1.5)

                elif key == '5':
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
                    print("\nEnter positions in radians (e.g., '0.785 1.57 -0.523' for servo_4,5,6):")
                    try:
                        values = input().strip().split()
                        if len(values) == 3:
                            positions = {
                                4: float(values[0]),
                                5: float(values[1]),
                                6: float(values[2])
                            }
                            node.send_position(positions)
                        else:
                            print("Error: Need 3 values")
                    except ValueError:
                        print("Error: Invalid input")
                    tty.setraw(sys.stdin.fileno())

                elif key == 'q' or key == '\x03':  # q or Ctrl+C
                    break

    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print("\n\nGripper Tester Stopped")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
