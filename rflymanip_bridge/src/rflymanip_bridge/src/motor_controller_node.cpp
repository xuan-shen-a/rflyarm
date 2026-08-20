/**
  ******************************************************************************
  * @file     motor_controller_node.cpp
  * @author   rxy
  * @date     2026/01/23
  * @brief    电机管理节点功能实现，用于管理关节电机
  ******************************************************************************
 **/

#include "rflymanip_bridge/motor_controller_node.hpp"

#include <chrono>
#include <memory>
#include <iomanip> 
#include <algorithm>
#include <sensor_msgs/msg/joint_state.hpp> 

using namespace std::chrono_literals;

/** @brief MotorController构造函数 **/ 
MotorControllerNode::MotorControllerNode(): Node("motor_controller_node")
{    
    // 从Launch文件中拿取电机ID (保持和电机硬件ID设定一致，可通过LKMotor拨码开关调整) 
    this->declare_parameter<std::vector<int64_t>>("motor_ids", {1, 2, 3});
    this->declare_parameter<std::vector<int64_t>>("motor_ratios", {10, 6, 6});
    std::vector<int64_t> motor_ids_param = this->get_parameter("motor_ids").as_integer_array();
    std::vector<int64_t> motor_ratios_param = this->get_parameter("motor_ratios").as_integer_array();

    motors.clear();
    for (size_t i = 0; i < motor_ids_param.size(); i++) 
    {
        uint8_t id = static_cast<uint8_t>(motor_ids_param[i]);
        uint8_t ratio = static_cast<uint8_t>(motor_ratios_param[i]);
        
        motors.push_back(std::make_shared<lkmotor::LKMotor>(id, ratio));
        RCLCPP_INFO(this->get_logger(), "Created motor ID: %d with ratio: %d", id, ratio);
    }

    control_params.resize(motor_ids_param.size());
    for(MotorControl_AllParams& p : control_params)
    {
        p.target_pos = 0.0f;
        p.target_vel = 0.0f;
        p.target_cur = 0.0f;
        p.zero_offset = 0.0f;  // 初始化零点偏移
        p.cmd_updated = false; // 无新目标时不下发控制指令
    }
    system_enabled = false;

    // 创建回调组：用于多线程执行
    control_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    read_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    std::string can_pub_topic = "/to_can_bus";
    std::string can_sub_topic = "/from_can_bus";

    std::string joint_cmd_topic = "/arm/joint_cmd";
    std::string state_cmd_topic = "/arm/state_cmd";
    std::string joint_states_topic = "/arm/joint_states";

    //发布
    motor_cmd_pub = this->create_publisher<can_msgs::msg::Frame>(
        can_pub_topic, 10);

    joint_state_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        joint_states_topic, 10);

    //订阅
    motor_state_sub = this->create_subscription<can_msgs::msg::Frame>(
        can_sub_topic, 10,
        std::bind(&MotorControllerNode::MotorStateCallback, this,
                  std::placeholders::_1));

    joint_cmd_sub = this->create_subscription<sensor_msgs::msg::JointState>(
        joint_cmd_topic, 10,
        std::bind(&MotorControllerNode::JointCmdCallback, this, std::placeholders::_1));

    sys_cmd_sub = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        state_cmd_topic, 10,
        std::bind(&MotorControllerNode::StateCmdCallback, this, std::placeholders::_1));

    // 控制命令定时器：20ms周期（50Hz），用于发送0xA4控制命令（独立线程）
    motor_control_timer = this->create_wall_timer(
        20ms, std::bind(&MotorControllerNode::controlTimerCallback, this),
        control_callback_group_);

    // 读取命令定时器：50ms周期（20Hz），用于发送0x92读取命令和广播角度（独立线程）
    // 降低读取频率可以完全避免与控制命令冲突，20Hz反馈频率已经足够实时
    motor_read_timer = this->create_wall_timer(
        50ms, std::bind(&MotorControllerNode::readTimerCallback, this),
        read_callback_group_);

    RCLCPP_INFO(this->get_logger(), "MotorControllerNode Initialized Successfully");
}

/** @brief MotorController析构函数 **/ 
MotorControllerNode::~MotorControllerNode()
{
    RCLCPP_INFO(this->get_logger(), "MotorControllerNode ShuttingDown");
}

/** @brief 关节指令回调函数 **/ 
void MotorControllerNode::JointCmdCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (msg->name.size() != msg->position.size()) 
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "JointState size mismatch. Name size: %zu, Pos size: %zu", 
            msg->name.size(), msg->position.size());
        return;
    }

    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        std::string name = msg->name[i];
        int id = -1;

        try {
            size_t last_underscore = name.find_last_of('_');
            if (last_underscore != std::string::npos) {
                std::string num_str = name.substr(last_underscore + 1);
                id = std::stoi(num_str);
            }
        }
        catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to parse motor ID from name '%s': %s", name.c_str(), e.what());
            continue;
        }

        if (id >= 1 && id <= (int)control_params.size()) 
        {
            int idx = id - 1;
            if (i < msg->position.size()) 
            {
                control_params[idx].target_pos = msg->position[i];
            }
            
            if (i < msg->velocity.size()) 
            {
                control_params[idx].target_vel = msg->velocity[i];
            } 
            else
            {
                control_params[idx].target_vel = 0.0f;
            }

            if (i < msg->effort.size())
            {
                control_params[idx].target_cur = msg->effort[i];
            }
            else
            {
                control_params[idx].target_cur = 0.0f;
            }

            // 收到新目标，标记需要下发一次控制指令
            control_params[idx].cmd_updated = true;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "BLOCKED: Motor ID %d is out of bounds! control_params.size() is %d", id, (int)control_params.size());
        }
    }
}

/** @brief 系统状态指令回调函数 **/
void MotorControllerNode::StateCmdCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
{
    system_enabled = false;  //确保操作电机时停止控制指令下发

    if (msg->data.size() < 2) 
    {
        RCLCPP_WARN(this->get_logger(), "StateCmd message requires at least 2 elements: [motor_id, state]");
        return;
    }

    uint8_t target_id = msg->data[0];
    uint8_t state = msg->data[1];
    
    if (target_id == 0)  //电机管理系统使能/失能
    {
        if (state == 1) {
            // 将所有电机的当前位置设为目标位置，避免突然运动
            for (size_t i = 0; i < motors.size(); i++) {
                float current_angle = motors[i]->GetAngleState().motor_angle;  // 单位：度（相对零点）
                control_params[i].target_pos = current_angle / (motors[i]->GetRatio() * 57.3f);  // 转换为弧度
                control_params[i].cmd_updated = true;  // 使能后下发一次保持当前位置的指令

                RCLCPP_INFO(this->get_logger(), "Motor %d: current_angle=%.2f deg (relative to zero), target_pos=%.4f rad",
                    motors[i]->GetID(), current_angle, control_params[i].target_pos);
            }

            // 发送使能命令到所有电机
            for (std::shared_ptr<lkmotor::LKMotor>& motor : motors) {
                can_msgs::msg::Frame enable_frame = motor->CmdControlEnable();
                enable_frame.header.stamp = this->get_clock()->now();
                motor_cmd_pub->publish(enable_frame);
            }

            RCLCPP_INFO(this->get_logger(), "System ENABLED - target positions set to current positions");
        } else if (state == 0) {
            // 失能所有电机：先发送电流环命令(力矩=0)，释放刹车，再发送失能命令
            for (std::shared_ptr<lkmotor::LKMotor>& motor : motors) {
                // 1. 先发送力矩为0的电流环命令
                can_msgs::msg::Frame torque_zero_frame = motor->CmdControlCur(0.0f);
                torque_zero_frame.header.stamp = this->get_clock()->now();
                motor_cmd_pub->publish(torque_zero_frame);

                rclcpp::sleep_for(std::chrono::milliseconds(10));

                // 2. 释放刹车
                can_msgs::msg::Frame brake_release_frame = motor->CmdControlBrake(true);
                brake_release_frame.header.stamp = this->get_clock()->now();
                motor_cmd_pub->publish(brake_release_frame);

                rclcpp::sleep_for(std::chrono::milliseconds(10));

                // 3. 发送失能命令
                can_msgs::msg::Frame disable_frame = motor->CmdControlDisable();
                disable_frame.header.stamp = this->get_clock()->now();
                motor_cmd_pub->publish(disable_frame);
            }
            system_enabled = false;
            RCLCPP_INFO(this->get_logger(), "System DISABLED - torque=0, brake released, motors disabled");
        }
        system_enabled = (state == 1);
        RCLCPP_INFO(this->get_logger(), "system_enabled is now: %s", system_enabled ? "true" : "false");
    }
    else  //单个电机操作
    {
        for (std::shared_ptr<lkmotor::LKMotor>& motor : motors)
        {
            if (motor->GetID() != target_id) continue;

            can_msgs::msg::Frame frame;
            bool valid_state = true;
            
            switch(state) {
                case 0:      //电机失能
                    frame = motor->CmdControlDisable();
                    break;
                case 1:      //电机使能
                    frame = motor->CmdControlEnable();
                    break;
                case 2:      //设置零点
                    // 硬件零点：将当前位置永久保存为零点（保存到ROM）
                    frame = motor->CmdControlSetzero();
                    valid_state = true;
                    RCLCPP_INFO(this->get_logger(), "Motor %d hardware zero set (saved to ROM)",
                        motor->GetID());

                    // 如果要使用软件零点（不修改硬件），取消下面注释并注释上面三行
                    // {
                    //     float current_angle = motor->GetAngleState().motor_angle;
                    //     int idx = motor->GetID() - 1;
                    //     control_params[idx].zero_offset = current_angle / (motor->GetRatio() * 57.3f);
                    //     RCLCPP_INFO(this->get_logger(), "Motor %d software zero set: offset=%.4f rad (%.2f deg)",
                    //         motor->GetID(), control_params[idx].zero_offset, current_angle);
                    // }
                    break;
                default:
                    valid_state = false;
                    break;
            }

            if (valid_state) {
                frame.header.stamp = this->get_clock()->now();
                motor_cmd_pub->publish(frame);
            }
        }
        
        std::string target_str = (target_id == 0) ? "" : ("Motor " + std::to_string(target_id));
        if (state == 0) {
            RCLCPP_INFO(this->get_logger(), "%s DISABLED via /arm/state_cmd", target_str.c_str());
        } 
        else if (state == 1) {
            RCLCPP_INFO(this->get_logger(), "%s ENABLED via /arm/state_cmd", target_str.c_str());
        } 
        else if (state == 2) {
            RCLCPP_INFO(this->get_logger(), "%s SET_ZERO via /arm/state_cmd", target_str.c_str());
        }
        else {
            RCLCPP_WARN(this->get_logger(), "Unknown system state: %d via /arm/state_cmd for %s", state, target_str.c_str());
        }
    }
}

/** @brief 电机CAN接收回调函数 **/
void MotorControllerNode::MotorStateCallback(const can_msgs::msg::Frame::SharedPtr msg)
{
    // 根据LK协议：命令和回复都使用 0x140 + ID(1~32)
    if (msg->id < 0x140 || msg->id > 0x140 + 32)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Unknown CAN frame ID: 0x%X", msg->id);
        return;
    }

    uint8_t motor_id = static_cast<uint8_t>(msg->id - 0x140);
    std::vector<std::shared_ptr<lkmotor::LKMotor>>::iterator it = std::find_if(
        motors.begin(),
        motors.end(),
        [motor_id](const std::shared_ptr<lkmotor::LKMotor>& motor)
        {
            return motor->GetID() == motor_id;
        });
    if (it == motors.end())
    {
        RCLCPP_WARN_ONCE(this->get_logger(), "Unknown motor ID: %d", motor_id);
        return;
    }

    if (!(*it)->ReplyUnpack(msg->data.data()))
    {
        RCLCPP_ERROR(this->get_logger(), "Motor %d data parse failed", motor_id);
    }
}

/** @brief 控制命令定时器回调函数（20ms周期） **/
void MotorControllerNode::controlTimerCallback()
{
    sendControlCommands();  // 仅发送0xA4控制命令
}

/** @brief 读取命令定时器回调函数（100ms周期） **/
void MotorControllerNode::readTimerCallback()
{
    publishJointStates();  // 发送0x92读取命令并广播角度
}

/** @brief 向上广播电机角度(deg) **/
void MotorControllerNode::publishJointStates()
{
    std_msgs::msg::Float32MultiArray msg;

    for (std::shared_ptr<lkmotor::LKMotor>& motor : motors)
    {
        can_msgs::msg::Frame angle_frame = motor->CmdReadMultiAngle();
        angle_frame.header.stamp = this->get_clock()->now();
        motor_cmd_pub->publish(angle_frame);
    }

    // 读取当前缓存的角度（MotorStateCallback会异步更新）
    for (std::shared_ptr<lkmotor::LKMotor>& motor : motors)
    {
        // 电机侧角度除以减速比得到负载侧角度（现实关节角度）
        float motor_angle_deg = motor->GetAngleState().motor_angle;  // 电机侧，单位：度
        float joint_angle_deg = motor_angle_deg / motor->GetRatio();  // 负载侧，单位：度
        float joint_angle_rad = joint_angle_deg / 57.2958f;  // 转换为弧度
        msg.data.push_back(joint_angle_rad);
    }

    joint_state_pub->publish(msg);
}

/** @brief 电机控制指令下发函数(仅在目标更新时下发一次) **/
void MotorControllerNode::sendControlCommands()
{
    if (!system_enabled) return;

    for (std::shared_ptr<lkmotor::LKMotor>& motor : motors)
    {
        int id = motor->GetID();

        MotorControl_AllParams& param = control_params[id - 1];

        // LK电机的位置/速度/电流指令均为锁存型：电机内部会一直保持最后一次收到的目标，
        // 因此没有新目标时不必重复下发，避免CAN总线被无意义的指令占满。
        if (!param.cmd_updated) continue;

        can_msgs::msg::Frame frame;
        bool valid_cmd = false;

        switch (motor->GetControlMode())
        {
            case lkmotor::ControlMode_Index::Control_Mode_MultiLoopPos2:
            {
                // 应用零点偏移
                float target_with_offset = param.target_pos - param.zero_offset;
                float deg = target_with_offset * motor->GetRatio() * 57.3f;

                // 速度限制：将关节速度(rad/s)转换为电机速度(dps)
                // 注意：这是最大速度限制，不是目标速度
                // 五次多项式会在轨迹执行过程中产生变化的速度值
                float max_dps = std::abs(param.target_vel) * motor->GetRatio() * 57.3f;

                // 避免零速度：LK电机协议中速度为0可能被理解为"无限制"
                // 使用1dps作为最小速度限制（约0.017 rad/s），防止电机突然高速运动
                if (max_dps < 1.0f) {
                    max_dps = 1.0f;
                }

                frame = motor->CmdControlPosMulti2(deg, max_dps);
                valid_cmd = true;

                RCLCPP_DEBUG(this->get_logger(),
                    "Motor ID %d: pos=%.4f rad (%.2f deg motor), vel=%.4f rad/s (%.2f dps motor)",
                    id, param.target_pos, deg, param.target_vel, max_dps);

                break;
            }
            case lkmotor::ControlMode_Index::Control_Mode_MultiLoopPos1:
            {
                // 兼容保留命令1（仅用于调试，正常不使用）
                float target_with_offset = param.target_pos - param.zero_offset;
                float deg = target_with_offset * motor->GetRatio() * 57.3f;
                frame = motor->CmdControlPosMulti1(deg);
                valid_cmd = true;

                RCLCPP_DEBUG(this->get_logger(),
                    "Motor ID %d: pos=%.4f rad (%.2f deg motor) [Command 1]",
                    id, param.target_pos, deg);

                break;
            }
            case lkmotor::ControlMode_Index::Control_Mode_Vel:
            {
                float dps = param.target_vel * motor->GetRatio() * 57.3f;
                float current_limit = (std::abs(param.target_cur) > 0.001f) ? std::abs(param.target_cur) : 30.0f;
                frame = motor->CmdControlVel(dps, current_limit);
                valid_cmd = true;
                break;
            }
            case lkmotor::ControlMode_Index::Control_Mode_Cur:
            {
                frame = motor->CmdControlCur(param.target_cur);
                valid_cmd = true;
                break;
            }
            default:
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Unknown Control Mode for Motor %d", id);
                break;
        }

        if(valid_cmd)
        {
            frame.header.stamp = this->get_clock()->now();
            motor_cmd_pub->publish(frame);
        }

        // 无论指令是否有效都清除标志，避免非法模式下反复告警
        param.cmd_updated = false;
    }
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorControllerNode>();

    // 使用多线程执行器，允许控制命令和读取命令并行执行
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}