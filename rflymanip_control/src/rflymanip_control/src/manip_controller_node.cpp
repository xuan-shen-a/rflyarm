/**
  ******************************************************************************
  * @file     manip_controller_node.cpp
  * @author   rxy
  * @date     2026/02/09
  * @brief    机械臂轨迹控制节点实现
  ******************************************************************************
 **/

#include "rflymanip_control/manip_controller_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

/** @brief ManipulatorController构造函数 **/
ManipulatorControllerNode::ManipulatorControllerNode(const std::string& node_name)
    : Node(node_name)
{
    this->declare_parameter<std::string>("target_pose_topic", "target_ee_pose");
    this->declare_parameter<std::string>("joint_state_topic", "/manip/joint_states");
    this->declare_parameter<std::string>("joint_cmd_topic", "/manip/joint_cmd");
    std::string target_pose_topic = this->get_parameter("target_pose_topic").as_string();
    std::string joint_state_topic = this->get_parameter("joint_state_topic").as_string();
    std::string joint_cmd_topic = this->get_parameter("joint_cmd_topic").as_string();

    this->declare_parameter<double>("control_rate", 50.0);
    this->declare_parameter<std::vector<double>>("joint_max_velocities", {0.5, 0.5, 0.5, 0.5, 0.5, 0.5});
    this->declare_parameter<std::vector<double>>("joint_max_accelerations", {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    this->declare_parameter<std::vector<double>>("joint_max_jerks", {5.0, 5.0, 5.0, 5.0, 5.0, 5.0});
    double control_rate = this->get_parameter("control_rate").as_double();
    joint_max_velocities_ = this->get_parameter("joint_max_velocities").as_double_array();
    joint_max_accelerations_ = this->get_parameter("joint_max_accelerations").as_double_array();
    joint_max_jerks_ = this->get_parameter("joint_max_jerks").as_double_array();

    this->declare_parameter<double>("time_safety_factor", 1.2);
    this->declare_parameter<double>("minimum_duration", 0.5);
    time_safety_factor_ = this->get_parameter("time_safety_factor").as_double();
    minimum_duration_ = this->get_parameter("minimum_duration").as_double();

    ik_solver_ = std::make_unique<IKSolver>();
    has_joint_state_ = false;
    trajectory_active_ = false;

    manip_cmd_msg_.name = {
        "joint_1", "joint_2", "joint_3",
        "joint_4", "joint_5", "joint_6"
    };
    manip_cmd_msg_.position.resize(6, 0.0);
    manip_cmd_msg_.velocity.resize(6, 0.0);
    manip_cmd_msg_.effort.resize(6, 0.0);

    //做为下位通信
    target_pose_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        target_pose_topic, 10,
        std::bind(&ManipulatorControllerNode::TargetPoseCallback, this, std::placeholders::_1));

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        joint_state_topic, 10,
        std::bind(&ManipulatorControllerNode::JointStateCallback, this, std::placeholders::_1));

    //做为上位通信
    manip_cmd_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        joint_cmd_topic, 10);

    //定时器相关    
    int64_t control_period_ms = std::max<int64_t>(1, static_cast<int64_t>(1000.0 / control_rate));
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(control_period_ms),
        std::bind(&ManipulatorControllerNode::ControllerTimerCallback, this));

    RCLCPP_INFO(this->get_logger(), "ManipulatorControllerNode Initialized Successfully");
}

/** @brief ManipulatorController析构函数 **/
ManipulatorControllerNode::~ManipulatorControllerNode()
{
    RCLCPP_INFO(this->get_logger(), "ManipulatorControllerNode ShuttingDown");
}

/* -------------------------------------------------- 插值相关 ----------------------------------------------- */
/** @brief 加入新的关节目标 **/
void ManipulatorControllerNode::addTrajectoryGoal(const std::vector<double>& joint_positions)
{
    auto is_same_goal = [this, &joint_positions](const std::vector<double>& goal)
    {
        if (joint_positions.size() != goal.size()) return false;

        for (size_t i = 0; i < joint_positions.size(); ++i)
        {
            if (std::abs(joint_positions[i] - goal[i]) > 0.01f) return false;
        }

        return true;
    };

    if (trajectory_active_ && is_same_goal(active_goal_)) return;

    for (const std::vector<double>& goal : trajectory_goals_)
    {
        if (is_same_goal(goal)) return;
    }

    trajectory_goals_.push_back(joint_positions);
    RCLCPP_INFO(this->get_logger(), "Added trajectory goal, queue size: %zu", trajectory_goals_.size());
}


/** @brief 开始队列中的下一段轨迹 **/
void ManipulatorControllerNode::startNextTrajectory()
{
    active_goal_ = trajectory_goals_.front();
    trajectory_goals_.pop_front();

    double duration = calculateTrajectoryDuration(current_joint_positions_, active_goal_);
    if (!trajectory_.setTrajectory(current_joint_positions_, active_goal_, duration))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to create quintic trajectory");
        return;
    }

    trajectory_start_time_ = this->now();
    trajectory_active_ = true;
    RCLCPP_INFO(this->get_logger(), "Started trajectory, duration: %.3f s", duration);
}


/** @brief 计算满足速度、加速度和jerk约束的统一运动时间 **/
double ManipulatorControllerNode::calculateTrajectoryDuration(
    const std::vector<double>& q0, const std::vector<double>& qf) const
{
    double duration = minimum_duration_;
    for (size_t i = 0; i < q0.size(); ++i)
    {
        double delta = std::abs(qf[i] - q0[i]);
        double velocity_time = 1.875 * delta / joint_max_velocities_[i];
        double acceleration_time = std::sqrt(5.7735 * delta / joint_max_accelerations_[i]);
        double jerk_time = std::cbrt(60.0 * delta / joint_max_jerks_[i]);

        duration = std::max(duration, std::max(velocity_time,
                            std::max(acceleration_time, jerk_time)) * time_safety_factor_);
    }

    return duration;
}


/** @brief 关节限位 **/
std::vector<double> ManipulatorControllerNode::clampJoints(
    const std::vector<double>& values,
    const std::vector<double>& lower,
    const std::vector<double>& upper) const
{
    std::vector<double> clamped(values.size());
    for (size_t i = 0; i < values.size(); ++i)
    {
        clamped[i] = std::clamp(values[i], lower[i], upper[i]);
    }
    return clamped;
}

/** @brief 发布轨迹采样关节指令 **/
void ManipulatorControllerNode::publishManipCmd(const QuinticTrajectorySample& sample)
{
    manip_cmd_msg_.header.stamp = this->now();
    manip_cmd_msg_.position = sample.q;
    manip_cmd_msg_.velocity = sample.dq;
    std::fill(manip_cmd_msg_.effort.begin(), manip_cmd_msg_.effort.end(), 0.0);
    manip_cmd_pub_->publish(manip_cmd_msg_);
}

/* -------------------------------------------------- 回调函数 ----------------------------------------------- */
/** @brief 控制定时器回调函数 **/
void ManipulatorControllerNode::ControllerTimerCallback()
{
    if (!has_joint_state_)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Waiting for /manip/joint_states before starting trajectory");
        return;
    }

    if (!trajectory_active_)
    {
        if (trajectory_goals_.empty()) return;
        startNextTrajectory();
        if (!trajectory_active_) return;
    }

    double elapsed = (this->now() - trajectory_start_time_).seconds();
    QuinticTrajectorySample sample = trajectory_.sample(elapsed);
    publishManipCmd(sample);

    if (trajectory_.isFinished(elapsed))
    {
        trajectory_active_ = false;
        RCLCPP_INFO(this->get_logger(), "Trajectory completed, remaining goals: %zu", trajectory_goals_.size());
    }
}


/** @brief 目标位姿回调函数 **/
void ManipulatorControllerNode::TargetPoseCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (msg->data.size() != 6)
    {
        RCLCPP_WARN(this->get_logger(), "Target pose message requires 6 elements");
        return;
    }

    std::vector<double> joint_positions = ik_solver_->solve(
        msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5]);

    for (double position : joint_positions)
    {
        if (!std::isfinite(position))
        {
            RCLCPP_WARN(this->get_logger(), "IK solution contains NaN or Inf");
            return;
        }
    }

    addTrajectoryGoal(clampJoints(joint_positions, joint_lower_limits_, joint_upper_limits_));
}


/** @brief 关节状态回调函数 **/
void ManipulatorControllerNode::JointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (msg->name.size() != msg->position.size())
    {
        RCLCPP_WARN(this->get_logger(), "Invalid joint state message received");
        return;
    }

    bool received[6] = {false, false, false, false, false, false};
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        size_t last_underscore = msg->name[i].find_last_of('_');
        if (last_underscore == std::string::npos) continue;

        try
        {
            int id = std::stoi(msg->name[i].substr(last_underscore + 1));
            if (id < 1 || id > 6) continue;

            current_joint_positions_[id - 1] = msg->position[i];
            received[id - 1] = true;
        }
        catch (const std::exception&)
        {
            continue;
        }
    }

    has_joint_state_ = received[0] && received[1] && received[2] &&
                       received[3] && received[4] && received[5];
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ManipulatorControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
