/**
  ******************************************************************************
  * @file     manipulator_ik_node.hpp
  * @author   rxy
  * @date     2026/02/09
  * @brief    机械臂IK控制节点头文件
  * @details 
  * 话题接口:
  *   订阅: target_ee_pose  (std_msgs/Float64MultiArray)
  *   发布: manip_cmd  (sensor_msgs/JointState, 6个关节)
  ******************************************************************************
 **/

#ifndef MANIPULATOR_IK_NODE_HPP
#define MANIPULATOR_IK_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include "rflymanip_control/ik_solver.hpp"
#include "rflymanip_control/quintic_trajectory.hpp"

#include <vector>
#include <memory>
#include <string>
#include <cmath>
#include <chrono>

class ManipulatorIkNode : public rclcpp::Node
{
public:
    // --- 构造/析构函数 ---
    ManipulatorIkNode(const std::string& node_name = "manipulator_ik_node");
    ~ManipulatorIkNode();

private:
    // --- 回调函数 ---
    /// 接收目标位姿，执行IK，仅更新缓存
    void targetPoseCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    /// 定时触发，读取缓存并下发命令
    void cmdTimerCallback();
    /// 接收电机关节状态（编码器坐标系）
    void armJointStatesCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    /// 接收舵机关节状态（编码器坐标系）
    void gripperJointStatesCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    // --- 工具函数 ---
    std::vector<double> clampJoints(
        const std::vector<double>& values,
        const std::vector<double>& lower,
        const std::vector<double>& upper);
    void publishManipCmd(const std::vector<double>& joint_positions, const std::vector<double>& joint_velocities);

    // --- 订阅 / 发布 / 定时器 ---
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr arm_joint_states_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr gripper_joint_states_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr manip_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr trajectory_debug_pub_;
    rclcpp::TimerBase::SharedPtr cmd_timer_;

    // --- IK求解器 ---
    std::unique_ptr<IKSolver> ik_solver_;

    // --- 轨迹插值器 ---
    QuinticTrajectory trajectory_;
    std::chrono::steady_clock::time_point trajectory_start_time_;
    bool trajectory_active_ = false;

    // --- 配置参数 ---
    double default_theta_rad_  = 0.0;   // 默认腕部旋转 (rad)
    double default_gripper_rad_ = 0.0;  // 默认夹爪开合 (rad)
    double trajectory_duration_ = 1.0;  // 轨迹时长 (秒) - 缩短以提高速度

    // 关节限位 (弧度) - 与 manip_logic_node.cpp 保持一致
    // Joint1: 底座旋转 [-π,    π   ]  Joint2: 大臂俯仰 [π/4,  2π/3]
    // Joint3: 小臂俯仰 [π/4,  5π/6 ]  Joint4: 腕部俯仰 [-π/2, π/2 ]
    // Joint5: 腕部旋转 [-π/2,  π/2 ]  Joint6: 夹爪开合 [0,    π/3 ]
    std::vector<double> joint_lower_limits_ = {-M_PI, M_PI/4, M_PI/4, -M_PI_2, -M_PI_2, 0.0};
    std::vector<double> joint_upper_limits_ = { M_PI, 2.0*M_PI/3, 5.0*M_PI/6, M_PI_2, M_PI_2, M_PI/3};

    // --- 状态缓存 (回调写入 / 定时器读取) ---
    std::vector<double> cmd_joint_positions_ = std::vector<double>(6, 0.0);
    std::vector<double> current_joint_positions_ = std::vector<double>(6, 0.0);  // 机械坐标系
    std::vector<double> current_encoder_positions_ = std::vector<double>(6, 0.0);  // 编码器坐标系
    bool has_valid_cmd_ = false;        // 是否已有一次有效的IK解
    bool has_valid_feedback_ = false;   // 是否已接收到关节反馈

    sensor_msgs::msg::JointState manip_cmd_msg_;
};

#endif 
