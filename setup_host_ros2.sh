#!/bin/bash
# ROS2 宿主机环境设置脚本
# 用途：启用宿主机与 Docker 容器之间的 ROS2 通信

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 设置 FastDDS 使用 UDP 传输（避免共享内存隔离问题）
export FASTRTPS_DEFAULT_PROFILES_FILE="${SCRIPT_DIR}/config/fastdds_udp_only.xml"

# ROS2 基本配置
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "✓ ROS2 宿主机环境已配置"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "  配置文件: ${FASTRTPS_DEFAULT_PROFILES_FILE}"
echo "  ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
echo "  传输方式: UDP only (兼容 Docker 容器)"
echo ""
echo "现在可以与 Docker 容器中的 ROS2 节点通信："
echo ""
echo "  # 查看容器内节点"
echo "  ros2 node list"
echo ""
echo "  # 发送命令到容器"
echo "  ros2 topic pub --once /arm/state_cmd std_msgs/msg/UInt8MultiArray \"{data: [0, 1]}\""
echo ""
echo "  # 接收容器数据"
echo "  ros2 topic echo /arm/joint_states"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
