/**
  ******************************************************************************
  * @file     manipulator_ik_node.cpp
  * @author   rxy
  * @date     2026/02/09
  * @brief    机械臂IK控制节点实现
  * @details
  *   数据流:
  *     订阅回调: target_pose -> IK求解 -> 更新 cmd_joint_positions_
  *     定时器:   按固定频率读取缓存 -> publishManipCmd() -> bridge层
  ******************************************************************************
 **/

#include "rflymanip_control/ik_node.hpp"
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

/**  @brief IK控制节点构造函数  **/
ManipulatorIkNode::ManipulatorIkNode(const std::string& node_name)
    : Node(node_name)
{
    RCLCPP_INFO(this->get_logger(), "初始化机械臂IK控制节点...");

    std::string target_pose_topic = "target_ee_pose";
    std::string manip_cmd_topic = "manip_cmd";

    ik_solver_ = std::make_unique<IKSolver>();

    manip_cmd_msg_.name.resize(6);
    manip_cmd_msg_.name[0] = "joint_1";
    manip_cmd_msg_.name[1] = "joint_2";
    manip_cmd_msg_.name[2] = "joint_3";
    manip_cmd_msg_.name[3] = "joint_4";
    manip_cmd_msg_.name[4] = "joint_5";
    manip_cmd_msg_.name[5] = "joint_6";
    manip_cmd_msg_.position.resize(6, 0.0);
    manip_cmd_msg_.velocity.resize(6, 0.0);
    manip_cmd_msg_.effort.resize(6, 0.0);
    
    target_pose_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        target_pose_topic, 10,
        std::bind(&ManipulatorIkNode::targetPoseCallback, this, std::placeholders::_1)
    );

    arm_joint_states_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/arm/joint_states", 10,
        std::bind(&ManipulatorIkNode::armJointStatesCallback, this, std::placeholders::_1)
    );

    gripper_joint_states_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/gripper/joint_states", 10,
        std::bind(&ManipulatorIkNode::gripperJointStatesCallback, this, std::placeholders::_1)
    );

    manip_cmd_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        manip_cmd_topic, 10
    );

    trajectory_debug_pub_ = this->create_publisher<std_msgs::msg::String>(
        "trajectory_debug", 10
    );

    cmd_timer_ = this->create_wall_timer(
        20ms, std::bind(&ManipulatorIkNode::cmdTimerCallback, this));

    RCLCPP_INFO(this->get_logger(), "IK节点初始化完成!");
}

/**  @brief IK控制节点析构函数  **/
ManipulatorIkNode::~ManipulatorIkNode()
{
    RCLCPP_INFO(this->get_logger(), "IK节点正在关闭...");
}

/**  @brief 目标位姿回调函数  **/
void ManipulatorIkNode::targetPoseCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (msg->data.size() != 6) return;

    const double x           = msg->data[0];  // mm
    const double y           = msg->data[1];  // mm
    const double z           = msg->data[2];  // mm
    const double alpha_rad   = msg->data[3];  // rad (末端俯仰角)
    const double theta_rad   = msg->data[4];  // rad (腕部旋转角)
    const double gripper_rad = msg->data[5];  // rad (夹爪开合角度)
    std::vector<double> joint_positions = ik_solver_->solve(
        x, y, z, alpha_rad, theta_rad, gripper_rad
    );

    bool valid = true;
    for (size_t i = 0; i < 6; ++i) {
        if (std::isnan(joint_positions[i]) || std::isinf(joint_positions[i])) {
            valid = false;
            RCLCPP_WARN(this->get_logger(),
                        "IK 解第 %zu 个关节为 NaN/Inf, 位姿不可达 [x=%.1f, y=%.1f, z=%.1f]mm",
                        i + 1, x, y, z);
            break;
        }
    }

    if (!valid) {
        return;
    }

    joint_positions = clampJoints(joint_positions, joint_lower_limits_, joint_upper_limits_);

    // 计算关节位置变化量
    double max_delta = 0.0;
    for (size_t i = 0; i < 6; ++i) {
        double delta = std::abs(joint_positions[i] - current_joint_positions_[i]);
        if (delta > max_delta) max_delta = delta;
    }

    // 固定轨迹时长
    double trajectory_duration = 3.0;  // 秒

    // 启动从当前位置到目标位置的五次多项式轨迹
    if (!trajectory_.setTrajectory(current_joint_positions_, joint_positions, trajectory_duration)) {
        RCLCPP_ERROR(this->get_logger(), "轨迹设置失败");
        return;
    }

    trajectory_start_time_ = std::chrono::steady_clock::now();
    trajectory_active_ = true;
    cmd_joint_positions_ = joint_positions;
    has_valid_cmd_ = true;

    RCLCPP_INFO(this->get_logger(),
                 "[%.1f, %.1f, %.1f]mm a=%.3frad -> 启动%.2fs轨迹 (max_delta=%.4frad)",
                 x, y, z, alpha_rad, trajectory_duration, max_delta);
}

/**  @brief 定时器回调函数  **/
void ManipulatorIkNode::cmdTimerCallback()
{
    if (!has_valid_cmd_) return;

    if (trajectory_active_) {
        // 计算轨迹时间
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = now - trajectory_start_time_;
        double t = elapsed.count();

        // 采样轨迹
        auto sample = trajectory_.sample(t);

        // 更新当前位置
        current_joint_positions_ = sample.q;

        // 发布轨迹调试信息到独立话题
        std_msgs::msg::String debug_msg;
        char buffer[512];
        snprintf(buffer, sizeof(buffer),
                 "t=%.3fs | pos=[%.4f, %.4f, %.4f, %.4f, %.4f, %.4f] | "
                 "vel=[%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
                 t,
                 sample.q[0], sample.q[1], sample.q[2], sample.q[3], sample.q[4], sample.q[5],
                 sample.dq[0], sample.dq[1], sample.dq[2], sample.dq[3], sample.dq[4], sample.dq[5]);
        debug_msg.data = buffer;
        trajectory_debug_pub_->publish(debug_msg);

        // 发布位置和速度
        publishManipCmd(sample.q, sample.dq);

        // 检查是否完成
        if (trajectory_.isFinished(t)) {
            trajectory_active_ = false;
            RCLCPP_INFO(this->get_logger(), "轨迹执行完成 - 停止发送命令（电机锁存保持位置）");
        }
    }
    // 轨迹完成后不再发送命令，依赖电机锁存特性保持最后位置
}

/**  @brief 控制指令打包函数  **/
void ManipulatorIkNode::publishManipCmd(const std::vector<double>& joint_positions, const std::vector<double>& joint_velocities)
{
    manip_cmd_msg_.header.stamp = this->now();
    for (int i = 0; i < 6; ++i) {
        manip_cmd_msg_.position[i] = joint_positions[i];
        manip_cmd_msg_.velocity[i] = joint_velocities[i];
    }

    std::fill(manip_cmd_msg_.effort.begin(), manip_cmd_msg_.effort.end(), 0.0);

    manip_cmd_pub_->publish(manip_cmd_msg_);
}

/**  @brief 接收电机关节状态（编码器坐标系） **/
void ManipulatorIkNode::armJointStatesCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (msg->data.size() != 3) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "armJointStates 消息应包含3个关节，实际收到 %zu 个", msg->data.size());
        return;
    }

    // 读取编码器坐标系角度（弧度）
    double encoder0 = msg->data[0];
    double encoder1 = msg->data[1];
    double encoder2 = msg->data[2];

    // 转换为机械坐标系（参考 manip_logic_node.cpp 中的转换公式反推）
    // manip_logic_node.cpp 中:
    //   motor_cmd_msg_.position[0] = (in0 - 0.0) * 1
    //   motor_cmd_msg_.position[1] = (in1 - PI/2) * 1
    //   motor_cmd_msg_.position[2] = (in2 + in1 - PI ) * -1
    // 反推得到:
    //   mechanical0 = encoder0 + 0.0 = encoder0
    //   mechanical1 = encoder1 + PI/2
    //   mechanical2 = encoder2 + mechanical1  (注意：依赖 mechanical1)
    current_encoder_positions_[0] = encoder0;
    current_encoder_positions_[1] = encoder1;
    current_encoder_positions_[2] = encoder2;

    current_joint_positions_[0] = encoder0;
    current_joint_positions_[1] = encoder1 + M_PI_2;
    current_joint_positions_[2] = M_PI - encoder2 - current_joint_positions_[1];

    // 检查是否已收到舵机反馈
    if (current_encoder_positions_[3] != 0.0 ||
        current_encoder_positions_[4] != 0.0 ||
        current_encoder_positions_[5] != 0.0) {
        has_valid_feedback_ = true;
    }
}

/**  @brief 接收舵机关节状态（编码器坐标系=机械坐标系） **/
void ManipulatorIkNode::gripperJointStatesCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (msg->data.size() != 3) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "gripperJointStates 消息应包含3个关节，实际收到 %zu 个", msg->data.size());
        return;
    }

    // 舵机的编码器坐标系 = 机械坐标系（无需转换）
    current_encoder_positions_[3] = msg->data[0];
    current_encoder_positions_[4] = msg->data[1];
    current_encoder_positions_[5] = msg->data[2];

    current_joint_positions_[3] = msg->data[0];
    current_joint_positions_[4] = msg->data[1];
    current_joint_positions_[5] = msg->data[2];

    // 检查是否已收到电机反馈
    if (current_encoder_positions_[0] != 0.0 ||
        current_encoder_positions_[1] != 0.0 ||
        current_encoder_positions_[2] != 0.0) {
        has_valid_feedback_ = true;
    }
}

/**  @brief 关节限位  **/
std::vector<double> ManipulatorIkNode::clampJoints(
    const std::vector<double>& values,
    const std::vector<double>& lower,
    const std::vector<double>& upper)
{
    std::vector<double> clamped(values.size());
    for (size_t i = 0; i < values.size(); ++i) {
        clamped[i] = std::clamp(values[i], lower[i], upper[i]);

        // 若达到限位则发出警告
        if (values[i] < lower[i]) {
            RCLCPP_WARN(this->get_logger(),
                "关节%zu 超出下限: %.4f rad < %.4f rad (已限制)",
                i + 1, values[i], lower[i]);
        } else if (values[i] > upper[i]) {
            RCLCPP_WARN(this->get_logger(),
                "关节%zu 超出上限: %.4f rad > %.4f rad (已限制)",
                i + 1, values[i], upper[i]);
        }
    }
    return clamped;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ManipulatorIkNode>();

    RCLCPP_INFO(node->get_logger(), "机械臂IK控制节点运行中...");

    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "运行异常: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
