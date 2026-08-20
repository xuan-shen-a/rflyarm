#include "rclcpp/rclcpp.hpp"
#include "rflymanip_bridge/manip_logic_node.hpp"
#include <cmath>
#include <algorithm>

const double PI = 3.14159265358979323846;

ManipLogicNode::ManipLogicNode(): Node("manip_logic_node")
{
    std::string manip_cmd_topic = "manip_cmd";
    std::string motor_cmd_topic = "/arm/joint_cmd";
    std::string servo_cmd_topic = "/gripper/joint_cmd";    

    manip_cmd_sub = this->create_subscription<sensor_msgs::msg::JointState>(
        manip_cmd_topic, 10,
        std::bind(&ManipLogicNode::ManipCmdCallback, this, std::placeholders::_1));

    motor_cmd_pub = this->create_publisher<sensor_msgs::msg::JointState>(
        motor_cmd_topic, 10);

    servo_cmd_pub = this->create_publisher<sensor_msgs::msg::JointState>(
        servo_cmd_topic, 10);

    // 初始化消息变量
    motor_cmd_msg_.name.resize(3);
    motor_cmd_msg_.position.resize(3);
    motor_cmd_msg_.velocity.resize(3);
    motor_cmd_msg_.effort.resize(3);

    servo_cmd_msg_.name.resize(3);
    servo_cmd_msg_.position.resize(3);
    servo_cmd_msg_.velocity.resize(3);
    servo_cmd_msg_.effort.resize(3);
    
    // 关节偏置
    joint_offsets_.resize(6);
    joint_offsets_[0] = 0.0;
    joint_offsets_[1] = PI/2;
    joint_offsets_[2] = 0.0;
    joint_offsets_[3] = 0.0;
    joint_offsets_[4] = 0.0;
    joint_offsets_[5] = 0.0;

    // 关节方向
    joint_directions_.resize(6, 1);
    joint_directions_[2]= -1;

    // 关节限位
    joint_min_limits_.resize(6);
    joint_min_limits_[0] = -PI;
    joint_min_limits_[1] = PI/4;
    joint_min_limits_[2] = PI/4;
    joint_min_limits_[3] = -PI/2;
    joint_min_limits_[4] = -PI/2;
    joint_min_limits_[5] = 0;

    joint_max_limits_.resize(6);
    joint_max_limits_[0] = PI;
    joint_max_limits_[1] = 2*PI/3;
    joint_max_limits_[2] = 5*PI/6;
    joint_max_limits_[3] = PI/2;
    joint_max_limits_[4] = PI/2;
    joint_max_limits_[5] = PI/3;

    RCLCPP_INFO(this->get_logger(), "ManipLogicNode Started Successfully!");
}

ManipLogicNode::~ManipLogicNode()
{
    RCLCPP_INFO(this->get_logger(), "ManipLogicNode ShuttingDown");
}

void ManipLogicNode::ManipCmdCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // position 是必需字段，必须恰好 6 个；velocity/effort 缺失时用 0 填充
    if(msg->name.size() != 6 || msg->position.size() != 6)
    {
        RCLCPP_ERROR(this->get_logger(),
            "Invalid message: need 6 names and 6 positions, got %zu names and %zu positions",
            msg->name.size(), msg->position.size());
        return;
    }

    // 安全取值：下标越界时返回默认值
    auto get_vel = [&](size_t i) -> double {
        return i < msg->velocity.size() ? msg->velocity[i] : 0.0;
    };
    // effort 语义因下游而异：
    //   关节1~3 (电机): 仅在速度/电流控制模式下有效，位置模式下忽略
    //   关节4~6 (舵机): 加速度编码 0~254，0 时舵机节点套用默认值 50
    auto get_eff = [&](size_t i) -> double {
        return i < msg->effort.size() ? msg->effort[i] : 0.0;
    };

    // 关节电机指令拆分（关节1、2、3）
    // 限位应用于输入逻辑位置 msg->position
    double in0 = std::clamp(msg->position[0], joint_min_limits_[0], joint_max_limits_[0]);
    double in1 = std::clamp(msg->position[1], joint_min_limits_[1], joint_max_limits_[1]);
    double in2 = std::clamp(msg->position[2], joint_min_limits_[2], joint_max_limits_[2]);

    // 速度下限保护：LK电机协议中速度为0可能被理解为"无限制"
    // 使用最小速度限制，防止电机突然高速运动
    const double min_velocity_threshold = 0.1;  // rad/s 

    motor_cmd_msg_.name[0] = msg->name[0];
    motor_cmd_msg_.position[0] = (in0 - joint_offsets_[0]) * joint_directions_[0];
    double vel0 = std::abs(get_vel(0));
    motor_cmd_msg_.velocity[0] = (vel0 > 0.0 && vel0 < min_velocity_threshold) ? min_velocity_threshold : vel0;
    motor_cmd_msg_.effort[0]   = get_eff(0);

    motor_cmd_msg_.name[1] = msg->name[1];
    motor_cmd_msg_.position[1] = (in1 - joint_offsets_[1]) * joint_directions_[1];
    double vel1 = std::abs(get_vel(1));
    motor_cmd_msg_.velocity[1] = (vel1 > 0.0 && vel1 < min_velocity_threshold) ? min_velocity_threshold : vel1;
    motor_cmd_msg_.effort[1]   = get_eff(1);

    motor_cmd_msg_.name[2] = msg->name[2];
    motor_cmd_msg_.position[2] = (in2 + in1 - PI - joint_offsets_[2]) * joint_directions_[2];
    // 电机3的速度需要减去关节2的速度，因为位置是 (in2 + in1)
    // 取绝对值，因为电机速度是最大速度限制（非负），方向由位置决定
    double vel2 = std::abs(get_vel(2) + get_vel(1));
    motor_cmd_msg_.velocity[2] = (vel2 > 0.0 && vel2 < min_velocity_threshold) ? min_velocity_threshold : vel2;
    motor_cmd_msg_.effort[2]   = get_eff(2);

    // 舵机指令拆分（关节4、5、6）
    for(int i=0; i<3; i++)
    {
        servo_cmd_msg_.name[i] = msg->name[i+3];
        double in_servo = std::clamp(msg->position[i+3], joint_min_limits_[i+3], joint_max_limits_[i+3]);
        servo_cmd_msg_.position[i] = (in_servo - joint_offsets_[i+3]) * joint_directions_[i+3];
        servo_cmd_msg_.velocity[i] = get_vel(i+3);
        servo_cmd_msg_.effort[i]   = get_eff(i+3);
    }

    motor_cmd_pub->publish(motor_cmd_msg_);
    servo_cmd_pub->publish(servo_cmd_msg_);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ManipLogicNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}