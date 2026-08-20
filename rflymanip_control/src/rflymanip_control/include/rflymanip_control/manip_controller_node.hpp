/**
  ******************************************************************************
  * @file     manip_controller_node.hpp
  * @author   rxy
  * @date     2026/02/09
  * @brief    机械臂轨迹控制节点头文件
  * @details
  *   数据流:
  *     订阅回调: target_pose -> IK求解 -> 加入关节目标队列
  *     订阅回调: joint_states -> 更新当前关节反馈
  *     定时器:   五次轨迹采样 -> publishManipCmd() -> bridge层
  ******************************************************************************
 **/

#ifndef MANIP_CONTROLLER_NODE_HPP
#define MANIP_CONTROLLER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "rflymanip_control/ik_solver.hpp"
#include "rflymanip_control/quintic_trajectory.hpp"

#include <cmath>
#include <deque>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

class ManipulatorControllerNode : public rclcpp::Node
{
public:
    ManipulatorControllerNode(const std::string& node_name = "manip_controller_node");
    ~ManipulatorControllerNode();

private:
    //回调函数
    void TargetPoseCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void JointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void ControllerTimerCallback();

    // --- 工具/辅助函数 ---
    std::unique_ptr<IKSolver> ik_solver_;
    QuinticTrajectory trajectory_;

    void addTrajectoryGoal(const std::vector<double>& joint_positions);
    void startNextTrajectory();
    double calculateTrajectoryDuration(const std::vector<double>& q0,
                                       const std::vector<double>& qf) const;
    std::vector<double> clampJoints(const std::vector<double>& values,
                                    const std::vector<double>& lower,
                                    const std::vector<double>& upper) const;
    void publishManipCmd(const QuinticTrajectorySample& sample);

    //订阅/发布/定时器
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr manip_cmd_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    //轨迹参数
    std::vector<double> joint_max_velocities_;
    std::vector<double> joint_max_accelerations_;
    std::vector<double> joint_max_jerks_;
    double time_safety_factor_;
    double minimum_duration_;
    double goal_tolerance_;

    //关节限位
    std::vector<double> joint_lower_limits_ = {-M_PI, 0.0 , 0.0 , -M_PI_2, -M_PI, 0.0};
    std::vector<double> joint_upper_limits_ = { M_PI, M_PI, M_PI,  M_PI_2,  M_PI, M_PI};

    //状态缓存
    std::vector<double> current_joint_positions_ = std::vector<double>(6, 0.0);
    std::vector<double> active_goal_ = std::vector<double>(6, 0.0);
    std::deque<std::vector<double>> trajectory_goals_;
    rclcpp::Time trajectory_start_time_;
    bool has_joint_state_;
    bool trajectory_active_;

    sensor_msgs::msg::JointState manip_cmd_msg_;
};

#endif
