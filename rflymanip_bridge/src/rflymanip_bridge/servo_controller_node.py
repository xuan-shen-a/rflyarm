#!/usr/bin/env python3
"""
******************************************************************************
* @file     servo_controller_node.py
* @author   rxy
* @date     2026/03/18
* @brief    HX-30HM 总线舵机控制节点
*
*  Topic 接口:
*    订阅: /gripper/joint_cmd  (sensor_msgs/JointState)
*         - name[i]     : 舵机名称, 格式 "joint_<id>", 如 "joint_4"
*         - position[i] : 目标位置 (弧度 rad, 相对于零点)
*         - velocity[i] : 目标角速度 (rad/s)
*         - effort[i]   : 加速度   (范围 0 ~ 254, 舵机原始加速度值, 无量纲)
*
*    发布: /gripper/joint_states (sensor_msgs/JointState)
*         - name[i]     : 舵机名称 "servo_<id>"
*         - position[i] : 当前位置 (弧度 rad, 相对于零点)
*         - velocity[i] : 当前速度 (暂未实现)
*
*  转换关系:
*    零点设置: 2048 (舵机原始编码中点)
*    舵机编码范围: 0~4096 对应 360° (2π rad)
*    因此: 1个编码单位 = 2π / 4096 ≈ 0.00153398 rad
*
*    angle_rad = (raw_value - 2048) * (2π / 4096.0)
*    raw_value = angle_rad * (4096.0 / 2π) + 2048
*
*  速度/加速度说明:
*    velocity: 输入单位 rad/s, 内部转换为 0~3400 编码值发送给舵机
*              转换系数: 1 编码单位 = 0.0015339808 rad/s
*              即: 编码值 1000 对应 1.5339808 rad/s
*    effort:   0~254  舵机原始加速度编码 (控制启停加速度, 非deg/s²)
*    默认值: velocity=1.2 rad/s (对应编码约782), effort=40 (当输入为0时使用)
******************************************************************************
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from servo_sdk import PortHandler, HxServoHandler, COMM_SUCCESS


class ServoControllerNode(Node):
    # 转换常数
    # HX-30HM舵机编码范围: 0~4096 对应 360° (2π rad)
    # 因此: 1个位置编码单位 = 2π / 4096 ≈ 0.00153398 rad
    SERVO_ZERO_POINT = 2048  # 舵机零点编码值 (中点)
    DEG_TO_RAW_RATIO = 4096.0 / 360.0  # 角度 → 原始值系数 (≈11.376)
    RAW_TO_DEG_RATIO = 360.0 / 4096.0  # 原始值 → 角度系数 (=0.0879)
    RAD_TO_RAW_RATIO = 4096.0 / (2 * math.pi)  # 弧度 → 原始值系数 (≈651.90)
    RAW_TO_RAD_RATIO = (2 * math.pi) / 4096.0  # 原始值 → 弧度系数 (≈0.00153398)

    # 速度转换常数
    # 舵机速度编码范围: 0~3400
    # 实测: 1个速度编码单位 = 0.0015339808 rad/s
    SPEED_ENCODE_TO_RAD_PER_SEC = 0.0015339808  # 速度编码 → rad/s
    SPEED_RAD_PER_SEC_TO_ENCODE = 1.0 / 0.0015339808  # rad/s → 速度编码 (≈651.898)

    # 舵机方向配置 (servo_id: direction)
    # direction = 1: 正向, direction = -1: 反向
    SERVO_DIRECTION = {
        4: 1,   # joint_4 正向
        5: 1,   # joint_5 正向
        6: -1,  # joint_6 反向 ⭐
    }

    # ———————————————— 构造函数 ————————————————
    def __init__(self):
        super().__init__('servo_conrtroller_node')
        
        #  参数声明
        port = '/dev/ttyACM0'

        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('servo_ids', [4, 5, 6])
        self.ids     = self.get_parameter('servo_ids').value
        baudrate     = self.get_parameter('baudrate').value

        self.port_handler  = PortHandler(port)
        self.servo_handler = HxServoHandler(self.port_handler)

        self.control_params = {
            sid: {'pos': 0, 'speed': 0, 'acc': 0}
            for sid in self.ids
        }
        self.system_enabled = False

        # 默认控制参数（内部使用编码值）
        self.config_speed = int(1.2 / 0.0015339808)  # 默认速度编码值，对应 1.2 rad/s ≈ 782
        self.config_speed_rad_s = 1.2  # 对应的 rad/s 值（用于日志显示）
        self.config_acc = 40  # 默认加速度编码值

        #  串口初始化
        if not self.port_handler.openPort():
            self.get_logger().fatal(f'Unable to open port: {port}')
            raise RuntimeError('Port open failed')

        if not self.port_handler.setBaudRate(baudrate):
            self.get_logger().fatal(f'Baudrate set failed: {baudrate}')
            raise RuntimeError('Baudrate set failed')

        self.get_logger().info(
            f'Serial Port Opened {port} @ {baudrate}bps, IDs: {self.ids}')

        #  舵机管理初始化
        for sid in self.ids:
            result, error = self.servo_handler.torqueEnable(sid)
            if result != COMM_SUCCESS:
                self.get_logger().warn(
                    f'Servo {sid} Servo Enable Failed: '
                    f'{self.servo_handler.getTxRxResult(result)}')
            else:
                self.get_logger().info(f'Servo {sid} Servo Enabled Successfully')

        #  订阅/发布
        self.joint_cmd_sub = self.create_subscription(
            JointState,
            '/gripper/joint_cmd',
            self.joint_cmd_callback,
            10)

        self.joint_state_pub = self.create_publisher(
            JointState,
            '/gripper/joint_states',
            10)

        # 定时器
        self.control_timer = self.create_timer(
            20.0 / 1000.0, self.timer_callback)

        self.get_logger().info('ServoControllerNode Initialized Successfully')

    # ——————————————— 析构函数 ———————————————
    def destroy_node(self):
        self.get_logger().info('ServoControllerNode ShuttingDown')

        for sid in self.ids:
            self.servo_handler.torqueDisable(sid)
        self.port_handler.closePort()
        super().destroy_node()

    # ——————————————— 关节指令回调 ———————————————
    def joint_cmd_callback(self, msg: JointState):
        self.system_enabled = True
        for i, name in enumerate(msg.name):
            try:
                sid = int(name.split('_')[-1])
            except (ValueError, IndexError):
                self.get_logger().warn(f'Failed to parse servo ID from name: {name}')
                continue

            if sid not in self.control_params:
                continue

            # 位置: 从弧度(rad)转换为原始值
            if i < len(msg.position):
                angle_rad = msg.position[i]
                # 应用方向配置并转换为原始值
                direction = self.SERVO_DIRECTION.get(sid, 1)
                raw_pos = (angle_rad * direction) * self.RAD_TO_RAW_RATIO + self.SERVO_ZERO_POINT
                self.control_params[sid]['pos'] = raw_pos
            else:
                self.control_params[sid]['pos'] = 0

            # 速度: 从 rad/s 转换为编码值 (0~3400)
            if i < len(msg.velocity) and msg.velocity[i] > 0:
                speed_rad_s = msg.velocity[i]
                speed_encode = int(speed_rad_s * self.SPEED_RAD_PER_SEC_TO_ENCODE)
                # 限制在有效范围内
                self.control_params[sid]['speed'] = max(0, min(speed_encode, 3400))
            else:
                self.control_params[sid]['speed'] = 0

            # 加速度: 保持原始编码值
            self.control_params[sid]['acc'] = msg.effort[i] if i < len(msg.effort) else 0

    # ———————————————— 定时器回调 ————————————————
    def timer_callback(self):
        self.send_control_commands()
        self.publish_joint_states()

    # ————————————— 舵机控制指令下发 ———————————————
    def send_control_commands(self):
        if not self.system_enabled:
            return

        # 逐个舵机将参数填入 GroupSyncWrite 缓冲区（不发送）
        for sid in self.ids:
            # control_params['pos'] 已经是原始编码值
            target_pos = int(self.control_params[sid]['pos'])

            # 使用用户指定的速度和加速度，如果为0则使用默认值
            target_speed = int(self.control_params[sid]['speed']) if self.control_params[sid]['speed'] > 0 else self.config_speed
            target_acc = int(self.control_params[sid]['acc']) if self.control_params[sid]['acc'] > 0 else self.config_acc

            ok = self.servo_handler.syncWritePosEx(
                sid,
                target_pos,
                target_speed,
                target_acc
            )
            if ok:
                self.get_logger().debug(
                    f'Updating servo ID {sid} to position {target_pos}',
                    throttle_duration_sec=1.0)

        # 一次广播发送
        result = self.servo_handler.GroupSyncWrite.txPacket()
        if result != COMM_SUCCESS:
            self.get_logger().warn(
                f'SyncWrite broadcast failed: {result}',
                throttle_duration_sec=1.0)

        self.servo_handler.GroupSyncWrite.clearParam()

    # ——————————————— 广播舵机位置 —————————————————
    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        for sid in self.ids:
            pos, result, error = self.servo_handler.readPos(sid)
            if result != COMM_SUCCESS:
                self.get_logger().warn(
                    f'Read Servo {sid} Position Failed',
                    throttle_duration_sec=2.0)
                continue

            # 转换为弧度(rad)
            direction = self.SERVO_DIRECTION.get(sid, 1)
            angle_rad = ((pos - self.SERVO_ZERO_POINT) * self.RAW_TO_RAD_RATIO) * direction

            msg.name.append(f'servo_{sid}')
            msg.position.append(angle_rad)

        if msg.name:
            self.joint_state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ServoControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
