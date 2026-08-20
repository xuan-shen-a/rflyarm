"""
******************************************************************************
* @file     motor_controller.launch.py
* @author   rxy
* @date     2026/02/01
* @brief    LK关节电机控制及逻辑分发节点启动文件
******************************************************************************
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 声明启动参数
    motor_ids_arg = DeclareLaunchArgument(
        'motor_ids',
        default_value='[1, 2, 3]',
        description='电机 ID 列表'
    )

    servo_port_arg = DeclareLaunchArgument(
        'servo_port',
        default_value='/dev/ttyACM0',
        description='舵机串口设备'
    )

    servo_ids_arg = DeclareLaunchArgument(
        'servo_ids',
        default_value='[4, 5, 6]',
        description='舵机 ID 列表'
    )

    # 启动控制节点
    motor_controller_node = Node(
        package='rflymanip_bridge',
        executable='motor_controller_node',
        name='motor_controller',
        output='screen',
        parameters=[{
            'motor_ids': LaunchConfiguration('motor_ids'),
        }]
    )

    manip_logic_node = Node(
        package='rflymanip_bridge',
        executable='manip_logic_node', 
        name='manip_logic_node',
        output='screen'
    )

    servo_controller_node = Node(
        package='rflymanip_bridge',
        executable='servo_controller_node.py',
        name='servo_controller_node',
        output='screen',
        parameters=[{
            'port':         LaunchConfiguration('servo_port'),
            'baudrate':     1000000,
            'servo_ids':    LaunchConfiguration('servo_ids'),
            'publish_rate': 50.0,
        }]
    )

    return LaunchDescription([
        motor_ids_arg,
        servo_port_arg,
        servo_ids_arg,
        motor_controller_node,
        manip_logic_node,
        servo_controller_node,
    ])