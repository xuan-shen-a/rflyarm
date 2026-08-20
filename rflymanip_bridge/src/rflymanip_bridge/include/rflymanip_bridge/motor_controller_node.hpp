/**
  ******************************************************************************
  * @file     motor_controller_node.hpp
  * @author   rxy
  * @date     2026/01/23
  * @brief    电机管理节点结构及接口
  ******************************************************************************
 **/

#ifndef MOTOR_CONTROLLER_NODE_HPP_
#define MOTOR_CONTROLLER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/frame.hpp"
#include "rflymanip_bridge/lkmotor_driver.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

struct MotorControl_AllParams
{
    float target_pos;
    float target_vel;
    float target_cur;
    float zero_offset;  // 软件零点偏移（弧度）
    bool  cmd_updated;  // 目标是否有更新，仅在为true时下发一次控制指令
};

class MotorControllerNode : public rclcpp::Node
{
public:
    MotorControllerNode(); 
    virtual ~MotorControllerNode();

private:
    // --- 回调函数 ---
    void MotorStateCallback(const can_msgs::msg::Frame::SharedPtr msg);
    void JointCmdCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void StateCmdCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
    void controlTimerCallback();         // 控制命令定时器（20ms）
    void readTimerCallback();            // 读取命令定时器（100ms）
    void sendControlCommands();
    void publishJointStates();

    // --- 订阅发布 ---
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr motor_cmd_pub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr joint_state_pub; // 广播各电机多圈角度(deg)
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr motor_state_sub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_sub;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sys_cmd_sub;
    rclcpp::TimerBase::SharedPtr motor_control_timer;  // 20ms定时器：发送控制命令
    rclcpp::TimerBase::SharedPtr motor_read_timer;     // 20ms定时器：发送读取命令

    // --- 回调组（用于多线程执行） ---
    rclcpp::CallbackGroup::SharedPtr control_callback_group_;  // 控制命令组
    rclcpp::CallbackGroup::SharedPtr read_callback_group_;     // 读取命令组

    // --- 电机相关 ---
    bool system_enabled;
    std::vector<std::shared_ptr<lkmotor::LKMotor>> motors;
    std::vector<int> motor_ids;
    std::vector<MotorControl_AllParams> control_params;
};

#endif  