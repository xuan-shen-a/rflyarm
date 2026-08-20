#ifndef MANIP_LOGIC_NODE_HPP
#define MANIP_LOGIC_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

struct ManipControl_AllParams
{
    float target_pos;
    float target_vel;
    float target_cur; 
};

class ManipLogicNode : public rclcpp::Node
{
public:
    ManipLogicNode(); 
    virtual ~ManipLogicNode();

private:
    // --- 回调函数 ---
    void ManipCmdCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    // --- 消息变量 ---
    sensor_msgs::msg::JointState motor_cmd_msg_;
    sensor_msgs::msg::JointState servo_cmd_msg_;

    std::vector<double> joint_offsets_;
    std::vector<int> joint_directions_;
    std::vector<double> joint_min_limits_;
    std::vector<double> joint_max_limits_;

    // --- 订阅发布 ---
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr motor_cmd_pub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr servo_cmd_pub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr manip_cmd_sub;
};

#endif  