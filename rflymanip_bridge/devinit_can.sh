#!/bin/bash
set -e

TARGET_BITRATE=1000000

setup_can_interface() {
    local DEV=$1

    #检查设备是否存在
    if ! ip link show "$DEV" > /dev/null 2>&1; then
        echo "ERROR: 设备 $DEV 不存在"
        return
    fi

    #检查设备是否开启
    local IS_UP="false"
    if ip link show "$DEV" | grep -q "state UP"; then
        IS_UP="true"
    fi

    #检查波特率
    local CURRENT_BITRATE=$(ip -d link show "$DEV" | grep -o "bitrate [0-9]*" | awk '{print $2}')
    if [ -z "$CURRENT_BITRATE" ]; then
        CURRENT_BITRATE=0
    fi

    if [ "$IS_UP" == "true" ]; then
        if [ "$CURRENT_BITRATE" == "$TARGET_BITRATE" ]; then
            echo "$DEV 已存在，跳过初始化"
            return
        else
            echo "$DEV 波特率不匹配，关闭接口"
            ip link set "$DEV" down
        fi
    else
        echo "$DEV 处于关闭状态"
    fi
    
    if ip link set "$DEV" up type can bitrate $TARGET_BITRATE; then
        echo "$DEV 启动成功"
    else
        echo "$DEV 启动失败"
    fi
}

setup_can_interface can0
echo "CAN通信检查完毕, 启动ROS2节点"

./start_brige.sh