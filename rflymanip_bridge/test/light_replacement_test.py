#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import sys
import termios
import tty
import threading

POS_INITIAL = [0.0, 0.2 , -0.2,  0.0,  0.0,   0.0]  # 示例初始位
POS_RELAY   = [0.0, 0.05,  0.4, 1.57,  0.0,   0.0]  # 示例中继位
POS_READY   = [0.0, 0.05,  0.1, 1.57,  0.0,   0.0]  # 示例换灯预备位
POS_CATCH   = [0.0, 0.05,  0.1, 1.57,  0.0,  -1.2]  # 示例换灯抓紧位
POS_ROTATE  = [0.0, 0.05,  0.1, 1.57,  1.57, -1.2]  # 示例换灯旋转位
POS_RELEASE = [0.0, 0.05,  0.1, 1.57,  1.57,   0.0]  # 示例换灯释放位
POS_END     = [0.0, 0.05,  0.4, 1.57,  0.0,  -1.2]  # 示例结束位

class LightReplacementTestNode(Node):
    def __init__(self):
        super().__init__('light_replacement_test_node')
        # 直接向底层控制器发送
        self.arm_pub = self.create_publisher(JointState, '/arm/joint_cmd', 10)
        self.gripper_pub = self.create_publisher(JointState, '/gripper/joint_cmd', 10)
        
        self.target_full_pos = list(POS_INITIAL)
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        self.get_logger().info('使用键盘按键执行动作: (1: 初始位, 2: 换灯位, 3: 抓紧位, 4: 旋转位, 5: 释放位, 6: 结束位, q: 退出)')

    def timer_callback(self):
        now = self.get_clock().now().to_msg()
        
        # 1. 发送机械臂指令 (前 3 个关节)
        arm_msg = JointState()
        arm_msg.header.stamp = now
        arm_msg.name = ['motor_1', 'motor_2', 'motor_3']
        arm_msg.position = self.target_full_pos[0:3]
        arm_msg.velocity = [0.0] * 3
        arm_msg.effort = [0.0] * 3
        self.arm_pub.publish(arm_msg)
        
        # 2. 发送夹爪/舵机指令 (后 3 个关节)
        # 位置: 弧度 rad, 速度: rad/s, 加速度: 编码值 (0~254)
        gripper_msg = JointState()
        gripper_msg.header.stamp = now
        gripper_msg.name = ['joint_4', 'joint_5', 'joint_6']
        gripper_msg.position = self.target_full_pos[3:6]  # 弧度
        gripper_msg.velocity = [1.2, 1.2, 1.2]  # rad/s
        gripper_msg.effort = [40.0, 40.0, 40.0]  # 加速度编码
        self.gripper_pub.publish(gripper_msg)

    def set_target(self, pos, label):
        self.target_full_pos = list(pos)
        self.get_logger().info(f'>> 切换至: {label}')

def get_key():
    settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    rclpy.init(args=args)
    node = LightReplacementTestNode()
    
    # 在独立线程中监听键盘，防止阻塞 ROS 循环
    def keyboard_listener():
        while rclpy.ok():
            key = get_key()
            if key == '1':
                node.set_target(POS_INITIAL, '初始位')
            elif key == '2':
                node.set_target(POS_RELAY, '中继位')
            elif key == '3':    
                node.set_target(POS_READY, '换灯位')
            elif key == '4':
                node.set_target(POS_CATCH, '抓紧位') 
            elif key == '5':
                node.set_target(POS_ROTATE, '旋转位') 
            elif key == '6':
                node.set_target(POS_RELEASE, '释放位') 
            elif key == '7':
                node.set_target(POS_END, '结束位') 
            elif key == 'q':
                node.get_logger().info('退出中...')
                break
        rclpy.shutdown()

    thread = threading.Thread(target=keyboard_listener)
    thread.start()

    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()
