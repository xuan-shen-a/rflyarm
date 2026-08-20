colcon build
source install/setup.bash

# 确保 ROS2 daemon 干净启动
echo "准备启动 ROS2 daemon..."
ros2 daemon stop > /dev/null 2>&1 || true
sleep 1
ros2 daemon start
echo "✓ ROS2 daemon 已启动"

ros2 launch ros2_socketcan socket_can_bridge.launch.xml receiver_interval_sec:=0.001 sender_timeout_sec:=0.001 > /dev/null 2>&1 &
echo "✓ SocketCAN节点启动完成"

ros2 launch rflymanip_bridge motor_controller.launch.py &
echo "✓ Motor/Servo Controller节点启动完成"

# 等待节点完全注册到 DDS 网络
echo "等待节点注册到 ROS2 网络..."
sleep 3

# 重启 daemon 确保节点发现缓存更新
echo "刷新节点发现缓存..."
ros2 daemon stop
sleep 1
ros2 daemon start
sleep 2

echo ""
echo "======================================"
echo "  ROS2 节点启动完成"
echo "======================================"
echo "可用节点列表:"
ros2 node list
echo ""
echo "提示: 如果宿主机看不到节点，请运行:"
echo "  ros2 daemon stop && ros2 daemon start"
echo "======================================"

# 自动使能
ros2 topic pub --once /arm/state_cmd std_msgs/msg/UInt8MultiArray "{data: [0, 1]}"

# 保持容器运行（不使用wait，因为ROS2节点是后台进程，wait会永久阻塞）
tail -f /dev/null