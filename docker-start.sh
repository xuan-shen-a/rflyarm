#!/bin/bash

# Docker环境快速启动脚本
#
# 使用方法:
#   source ./docker-start.sh   (推荐 - ROS2环境变量会在当前终端生效)
#   ./docker-start.sh          (普通模式 - 仅启动容器)

# 检测是否被source执行
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    SOURCED=false
else
    SOURCED=true
fi

echo "======================================"
echo "  RFly Manipulator Docker环境启动"
echo "======================================"
echo ""

# 检查Docker Compose版本
if ! command -v docker &> /dev/null; then
    echo "❌ 错误: Docker未安装"
    exit 1
fi

echo "✓ Docker版本: $(docker --version)"

if docker compose version &> /dev/null; then
    echo "✓ Docker Compose版本: $(docker compose version)"
    COMPOSE_CMD="docker compose"
else
    echo "❌ 错误: Docker Compose V2未安装"
    echo "请升级Docker或使用 'docker compose' 命令"
    exit 1
fi

echo ""
echo "✓ Docker已就绪"
echo ""

# 询问是否需要编译
echo "======================================"
read -p "是否需要编译代码？[y/N]: " compile_choice
echo ""

if [[ "$compile_choice" =~ ^[Yy]$ ]]; then
    echo "🚀 确保容器运行中..."
    $COMPOSE_CMD up -d
    echo ""

    echo "🔨 编译 rflymanip_bridge..."
    docker exec rflymanip_motor bash -c "
        cd /workspace/rflymanip_bridge &&
        colcon build --packages-select rflymanip_bridge
    "
    echo ""
    echo "✓ rflymanip_bridge 编译完成"
    echo ""

    echo "🔨 编译 rflymanip_control..."
    docker exec rflymanip_control bash -c "
        cd /workspace/rflymanip_control &&
        colcon build
    "
    echo ""
    echo "✓ rflymanip_control 编译完成"
    echo ""

    read -p "是否重启容器使更改生效？[y/N]: " restart_choice
    if [[ "$restart_choice" =~ ^[Yy]$ ]]; then
        echo ""
        echo "🔄 重启容器..."
        $COMPOSE_CMD restart
        echo "✓ 容器已重启"
    fi
    echo ""
fi

echo "======================================"
echo "  启动选项"
echo "======================================"
echo "1. 启动服务+当前终端配置ROS2+新终端显示日志 "
echo "2. 停止所有服务"
echo "3. 查看运行状态"
echo "4. 进入motor-test容器"
echo "5. 进入control-test容器"
echo "0. 退出"
echo ""

read -p "请选择操作 [0-5]: " choice

case $choice in
    1)
        echo ""
        echo "🧹 停止容器内可能存在的ROS2进程..."
        docker exec rflymanip_motor bash -c 'killall -9 motor_controller_node manip_logic_node python3 2>/dev/null' || true
        docker exec rflymanip_control bash -c 'killall -9 python3 2>/dev/null' || true
        echo "✓ ROS2进程已清理"
        echo ""
        echo "⏹️  停止服务..."
        $COMPOSE_CMD down
        echo "✓ 服务已停止"
        echo ""
        echo "🚀 启动Docker容器..."
        $COMPOSE_CMD up -d
        echo "✓ 容器已启动"
        echo ""
        $COMPOSE_CMD ps
        echo ""

        if [ "$SOURCED" = true ]; then
            echo "📡 配置ROS2与Docker通信..."
            SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
            export FASTRTPS_DEFAULT_PROFILES_FILE="${SCRIPT_DIR}/config/fastdds_udp_only.xml"
            export ROS_DOMAIN_ID=0
            export ROS_LOCALHOST_ONLY=0
            echo "✓ ROS2通信已配置（当前终端）"
            echo ""

            # 在新终端中显示日志
            echo "📋 在新终端中显示日志..."
            if command -v gnome-terminal &> /dev/null; then
                gnome-terminal -- bash -c "
                    cd '$SCRIPT_DIR' && docker compose logs -f motor-test
                    echo ''
                    echo '🧹 清理容器内的ROS2进程...'
                    docker exec rflymanip_motor bash -c 'killall -9 motor_controller_node manip_logic_node python3 2>/dev/null'
                    echo '✓ ROS2进程已清理'
                    exec bash
                "
            elif command -v xterm &> /dev/null; then
                xterm -e "
                    cd '$SCRIPT_DIR' && docker compose logs -f motor-test
                    echo ''
                    echo '🧹 清理容器内的ROS2进程...'
                    docker exec rflymanip_motor bash -c 'killall -9 motor_controller_node manip_logic_node python3 2>/dev/null'
                    echo '✓ ROS2进程已清理'
                    exec bash
                " &
            else
                echo "⚠️  未找到终端模拟器（gnome-terminal/xterm），无法自动打开日志窗口"
                echo "   手动查看日志: docker compose logs -f motor-test"
            fi

            echo ""
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo "  🎉 启动完成！现在可以直接使用ROS2命令"
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo ""
            echo "测试命令:"
            echo "  ros2 node list"
            echo "  ros2 topic list"
            echo "  ros2 topic pub --once /arm/state_cmd std_msgs/msg/UInt8MultiArray \"{data: [0, 1]}\""
        else
            echo "⚠️  提示: 脚本未被source执行，ROS2环境变量未配置"
            echo ""
            echo "如需在当前终端使用ROS2命令，请使用:"
            echo "  source ./docker-start.sh"
            echo ""
            echo "或手动配置环境:"
            echo "  source ~/rflycontrol/setup_host_ros2.sh"
        fi
        echo ""
        echo "其他操作:"
        echo "  查看日志: docker compose logs -f motor-test"
        echo "  进入容器: docker exec -it rflymanip_motor bash"
        echo "  停止服务: docker compose down"
        echo ""
        ;;
    2)
        echo ""
        echo "⏹️  停止所有服务..."
        echo "🧹 清理容器内的ROS2进程..."
        docker exec rflymanip_motor bash -c 'killall -9 motor_controller_node manip_logic_node python3 2>/dev/null' || true
        docker exec rflymanip_control bash -c 'killall -9 python3 2>/dev/null' || true
        echo "✓ ROS2进程已清理"
        echo ""
        $COMPOSE_CMD down
        echo "✓ 服务已停止"
        ;;
    3)
        echo ""
        echo "📊 容器运行状态:"
        $COMPOSE_CMD ps
        echo ""
        echo "Docker容器:"
        docker ps | grep -E "rflymanip|CONTAINER"
        ;;
    4)
        echo ""
        echo "🔧 进入motor-test容器..."
        echo "提示: 输入 'exit' 退出容器"
        docker exec -it rflymanip_motor bash
        ;;
    5)
        echo ""
        echo "🔧 进入control-test容器..."
        echo "提示: 输入 'exit' 退出容器"
        docker exec -it rflymanip_control bash
        ;;
    0)
        echo "退出"
        exit 0
        ;;
    *)
        echo "❌ 无效选择"
        exit 1
        ;;
esac
