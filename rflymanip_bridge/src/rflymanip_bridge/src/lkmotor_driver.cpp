/**
  ******************************************************************************
  * @file     lkmotor_driver.cpp
  * @author   rxy
  * @date     2026/01/28
  * @brief    LK关节电机驱动具体函数功能实现
  ******************************************************************************
 **/

#include "rflymanip_bridge/lkmotor_driver.hpp"

namespace lkmotor {
    /* -------------------------------------------------- 构造/析构 ----------------------------------------------- */
    /** @brief LKMotor 构造函数 **/
    LKMotor::LKMotor(uint8_t motor_id, uint8_t motor_ratio) : can_id(motor_id), ratio(motor_ratio)
    {
        control_mode = ControlMode_Index::Control_Mode_MultiLoopPos2;  // 改为多圈位置闭环2
        std::memset(&total_state, 0, sizeof(total_state));
        std::memset(&work_state, 0, sizeof(work_state));
        std::memset(&current_state, 0, sizeof(current_state));
        std::memset(&param_state, 0, sizeof(param_state));
        std::memset(&encoder_state, 0, sizeof(encoder_state));
        std::memset(&angle_state, 0, sizeof(angle_state));
    }

    /* -------------------------------------------------- 打包 ----------------------------------------------- */
    /** @brief 控制命令: 电机使能 0x88 **/ 
    can_msgs::msg::Frame LKMotor::CmdControlEnable() {
        can_msgs::msg::Frame frame;
        frame.id = Get_ID_ToCmd();
        frame.dlc = 8;
        std::memset(frame.data.data(), 0, 8);
        frame.data[0] = static_cast<uint8_t>(CommandType_Index::Communication_Type_MOTOR_RUN);
        return frame;
    }

    /** @brief 控制命令: 电机失能 0x80 **/
    can_msgs::msg::Frame LKMotor::CmdControlDisable() {
        can_msgs::msg::Frame frame;
        frame.id = Get_ID_ToCmd();
        frame.dlc = 8;
        std::memset(frame.data.data(), 0, 8);
        frame.data[0] = static_cast<uint8_t>(CommandType_Index::Communication_Type_MOTOR_OFF);
        return frame;
    }
    
    /** @brief 控制命令: 电机停止 0x81 **/
    can_msgs::msg::Frame LKMotor::CmdControlStop() {
        can_msgs::msg::Frame frame;
        frame.id = Get_ID_ToCmd();
        frame.dlc = 8;
        std::memset(frame.data.data(), 0, 8);
        frame.data[0] = static_cast<uint8_t>(CommandType_Index::Communication_Type_MOTOR_STOP);
        return frame;
    }

    /** @brief 控制命令: 抱闸控制 0x8C **/
    can_msgs::msg::Frame LKMotor::CmdControlBrake(bool release) {
        can_msgs::msg::Frame frame;
        frame.id = Get_ID_ToCmd();
        frame.dlc = 8;
        std::memset(frame.data.data(), 0, 8);
        frame.data[0] = static_cast<uint8_t>(CommandType_Index::Communication_Type_BRAKE_OPEN);
        frame.data[1] = release ? 0x01 : 0x00;  // 0x01=释放, 0x00=锁死
        return frame;
    }

    /** @brief 控制命令: 设置零点 0x19 **/
    can_msgs::msg::Frame LKMotor::CmdControlSetzero() {
        can_msgs::msg::Frame frame;
        frame.id = Get_ID_ToCmd();
        frame.dlc = 8;
        std::memset(frame.data.data(), 0, 8);
        frame.data[0] = static_cast<uint8_t>(CommandType_Index::Communication_Type_SET_ZERO_ROM);
        return frame;
    }

    /** @brief 控制命令: 电流闭环 0xA1 **/
    can_msgs::msg::Frame LKMotor::CmdControlCur(float current) {
        can_msgs::msg::Frame frame;
        frame.id = Get_ID_ToCmd();
        frame.dlc = 8;
        std::memset(frame.data.data(), 0, 8);
        
        frame.data[0] = static_cast<uint8_t>(CommandType_Index::Communication_Type_CUR_LOOP);
        
        int16_t param = Cur_To_Int16(current);
        WriteInt16(&frame.data[4], param);
        return frame;
    }

    /** @brief 控制命令: 速度闭环 0xA2 **/
    can_msgs::msg::Frame LKMotor::CmdControlVel(float dps, float current_limit) {
        can_msgs::msg::Frame frame;
        frame.id = Get_ID_ToCmd();
        frame.dlc = 8;
        std::memset(frame.data.data(), 0, 8);
        
        frame.data[0] = static_cast<uint8_t>(CommandType_Index::Communication_Type_VEL_LOOP);
        
        int16_t limit = Cur_To_Int16(current_limit);
        WriteInt16(&frame.data[2], limit);

        int32_t param = Vel_To_Int32(dps);
        WriteInt32(&frame.data[4], param);

        return frame;
    }

    /** @brief 控制命令: 多圈位置闭环1 0xA3 **/
    can_msgs::msg::Frame LKMotor::CmdControlPosMulti1(float deg) {
        can_msgs::msg::Frame frame;
        frame.id = Get_ID_ToCmd();
        frame.dlc = 8;
        std::memset(frame.data.data(), 0, 8);
        
        frame.data[0] = static_cast<uint8_t>(CommandType_Index::Communication_Type_POS_MULTI_LOOP_1);
        
        int32_t param = Pos_To_Int32(deg);
        WriteInt32(&frame.data[4], param);
        return frame;
    }

    /** @brief 控制命令: 多圈位置闭环2 0xA4 **/
    can_msgs::msg::Frame LKMotor::CmdControlPosMulti2(float deg, float max_dps) {
        can_msgs::msg::Frame frame;
        frame.id = Get_ID_ToCmd();
        frame.dlc = 8;
        std::memset(frame.data.data(), 0, 8);
        
        frame.data[0] = static_cast<uint8_t>(CommandType_Index::Communication_Type_POS_MULTI_LOOP_2);
        
        uint16_t maxSpeed = VelLim_To_Uint16(max_dps);
        WriteInt16(&frame.data[2], maxSpeed);
        
        int32_t param = Pos_To_Int32(deg);
        WriteInt32(&frame.data[4], param);
        return frame;
    }

    /** @brief 读取命令: 多圈角度 0x92 **/
    can_msgs::msg::Frame LKMotor::CmdReadMultiAngle() {
        can_msgs::msg::Frame frame;
        frame.id = Get_ID_ToCmd();
        frame.dlc = 8;
        std::memset(frame.data.data(), 0, 8);
        frame.data[0] = static_cast<uint8_t>(CommandType_Index::Communication_Type_READ_MULTI_ANGLE);
        return frame;
    }

    /** @brief 读取命令: 单圈角度 0x94 **/
    can_msgs::msg::Frame LKMotor::CmdReadSingleAngle() {
        can_msgs::msg::Frame frame;
        frame.id = Get_ID_ToCmd();
        frame.dlc = 8;
        std::memset(frame.data.data(), 0, 8);
        frame.data[0] = static_cast<uint8_t>(CommandType_Index::Communication_Type_READ_SINGLE_ANGLE);
        return frame;
    }

    /* -------------------------------------------------- 解包 ----------------------------------------------- */ 
    namespace // 解包用私有命名空间
    {   
        /** @brief 回复类型枚举 */ 
        enum class ReplyType : uint8_t 
        {
            MotorTotalState,
            MotorWorkState,
            MotorCurrentState,
            MotorEncoderState,
            MotorAngleState,
            MotorParamState,
            Feedback,
            Unknown
        };

        //功能函数枚举 
        template<typename... Ts>
        bool is_one_of(CommandType_Index cmd, Ts... list) { return ((cmd == list) || ...); }
        bool IsType_MotorTotalState(CommandType_Index cmd);
        bool IsType_MotorWorkState(CommandType_Index cmd);
        bool IsType_MotorCurrentState(CommandType_Index cmd);
        bool IsType_MotorParamState(CommandType_Index cmd);
        bool IsType_MotorEncoderState(CommandType_Index cmd);
        bool IsType_MotorAngleState(CommandType_Index cmd);
        bool IsType_Feedback(CommandType_Index cmd);

        /** @brief 回复类型分类函数 */ 
        ReplyType ReplyTypeClassify(uint8_t rx_data) 
        {
            CommandType_Index cmd = static_cast<CommandType_Index>(rx_data);
            if (IsType_MotorTotalState(cmd)) return ReplyType::MotorTotalState;
            else if (IsType_MotorWorkState(cmd)) return ReplyType::MotorWorkState;
            else if (IsType_MotorCurrentState(cmd)) return ReplyType::MotorCurrentState;
            else if (IsType_MotorParamState(cmd)) return ReplyType::MotorParamState;
            else if (IsType_MotorEncoderState(cmd)) return ReplyType::MotorEncoderState;
            else if (IsType_MotorAngleState(cmd)) return ReplyType::MotorAngleState;
            else if (IsType_Feedback(cmd)) return ReplyType::Feedback;
            else return ReplyType::Unknown;
        }

        /** @brief 总状态判断 */ 
        bool IsType_MotorTotalState(CommandType_Index cmd) 
        {
            return is_one_of(cmd, 
                CommandType_Index::Communication_Type_READ_STATUS_1, 
                CommandType_Index::Communication_Type_CLEAR_ERROR); 
        }       

        /** @brief 工作状态判断 */ 
        bool IsType_MotorWorkState(CommandType_Index cmd) 
        {
            return is_one_of(cmd, 
                CommandType_Index::Communication_Type_READ_STATUS_2, 
                CommandType_Index::Communication_Type_CUR_LOOP,
                CommandType_Index::Communication_Type_VEL_LOOP,
                CommandType_Index::Communication_Type_POS_MULTI_LOOP_1,
                CommandType_Index::Communication_Type_POS_MULTI_LOOP_2,
                CommandType_Index::Communication_Type_POS_SINGLE_LOOP_1,
                CommandType_Index::Communication_Type_POS_SINGLE_LOOP_2,
                CommandType_Index::Communication_Type_POS_INC_LOOP_1,
                CommandType_Index::Communication_Type_POS_INC_LOOP_2);
        }

        /** @brief 电流状态判断 */ 
        bool IsType_MotorCurrentState(CommandType_Index cmd) 
        {
            return is_one_of(cmd, 
                CommandType_Index::Communication_Type_READ_STATUS_3);
        }

        /** @brief 参数状态判断 */ 
        bool IsType_MotorParamState(CommandType_Index cmd) 
        {
            return is_one_of(cmd, 
                CommandType_Index::Communication_Type_READ_PARAM,
                CommandType_Index::Communication_Type_WRITE_PARAM);
        }

        /** @brief 编码器状态判断 */ 
        bool IsType_MotorEncoderState(CommandType_Index cmd) 
        {
            return is_one_of(cmd, 
                CommandType_Index::Communication_Type_READ_ENCODER,
                CommandType_Index::Communication_Type_SET_ZERO_ROM);
        }

        /** @brief 角度状态判断 */ 
        bool IsType_MotorAngleState(CommandType_Index cmd) 
        {
            return is_one_of(cmd, 
                CommandType_Index::Communication_Type_READ_MULTI_ANGLE,
                CommandType_Index::Communication_Type_READ_SINGLE_ANGLE);
        }

        /** @brief 反馈状态判断 */ 
        bool IsType_Feedback(CommandType_Index cmd) 
        {
            return is_one_of(cmd, 
                CommandType_Index::Communication_Type_MOTOR_OFF,
                CommandType_Index::Communication_Type_MOTOR_STOP,
                CommandType_Index::Communication_Type_MOTOR_RUN,
                CommandType_Index::Communication_Type_SET_POS_RAM);
        }
    }
    
    /** @brief 回复报文解包 **/ 
    bool LKMotor::ReplyUnpack(const uint8_t* rx_data) {
        switch (ReplyTypeClassify(rx_data[0])) {
            case ReplyType::MotorTotalState:
                total_state.temperature = static_cast<int8_t>(rx_data[1]);
                total_state.voltage = static_cast<int16_t>(rx_data[2] | (rx_data[3] << 8)) * 0.01f;   // 0.01V/LSB
                total_state.current = static_cast<int16_t>(rx_data[4] | (rx_data[5] << 8)) * 0.01f;   // 0.01A/LSB
                total_state.motor_state = rx_data[6];
                total_state.error_state = rx_data[7];
                break;
            case ReplyType::MotorWorkState:
                work_state.temperature = static_cast<int8_t>(rx_data[1]);
                work_state.iq = Int16_To_Cur(static_cast<int16_t>(rx_data[2] | (rx_data[3] << 8)));
                work_state.speed = Int16_To_Vel(static_cast<int16_t>(rx_data[4] | (rx_data[5] << 8)));
                work_state.encoder = static_cast<uint16_t>(rx_data[6] | (rx_data[7] << 8));
                break;
            case ReplyType::MotorCurrentState:
                current_state.temperature = static_cast<int8_t>(rx_data[1]);
                current_state.current_a = Int16_To_Cur(static_cast<int16_t>(rx_data[2] | (rx_data[3] << 8)));
                current_state.current_b = Int16_To_Cur(static_cast<int16_t>(rx_data[4] | (rx_data[5] << 8)));
                current_state.current_c = Int16_To_Cur(static_cast<int16_t>(rx_data[6] | (rx_data[7] << 8)));
                break;
            case ReplyType::MotorParamState:
                //待补充
                break;
            case ReplyType::MotorEncoderState:
                if(rx_data[0] == static_cast<uint8_t>(CommandType_Index::Communication_Type_READ_ENCODER)) //编码器
                {
                    encoder_state.encoder = static_cast<uint16_t>(rx_data[2] | (rx_data[3] << 8));
                    encoder_state.encoder_raw = static_cast<uint16_t>(rx_data[4] | (rx_data[5] << 8));
                    encoder_state.encoder_offset = static_cast<uint16_t>(rx_data[6] | (rx_data[7] << 8));
                }
                else if(rx_data[0] == static_cast<uint8_t>(CommandType_Index::Communication_Type_SET_ZERO_ROM)) //写零点
                {
                    encoder_state.encoder_offset = static_cast<uint16_t>(rx_data[6] | (rx_data[7] << 8));
                }
                break;
            case ReplyType::MotorAngleState:
                if(rx_data[0] == static_cast<uint8_t>(CommandType_Index::Communication_Type_READ_MULTI_ANGLE)) //多圈角度
                {
                    // 读取7字节数据组成56位有符号整数，DATA[1]-DATA[7] 对应 motorAngle 字节 0-6
                    int64_t raw_angle = static_cast<int64_t>(
                        (static_cast<uint64_t>(rx_data[1]) << 0 ) |
                        (static_cast<uint64_t>(rx_data[2]) << 8 ) |
                        (static_cast<uint64_t>(rx_data[3]) << 16) |
                        (static_cast<uint64_t>(rx_data[4]) << 24) |
                        (static_cast<uint64_t>(rx_data[5]) << 32) |
                        (static_cast<uint64_t>(rx_data[6]) << 40) |
                        (static_cast<uint64_t>(rx_data[7]) << 48));

                    // 符号扩展：56位有符号数转64位，如果bit 55为1（负数），需要将bit 56-63设为1
                    if (raw_angle & 0x0080000000000000LL) {
                        raw_angle |= 0xFF00000000000000LL;
                    }

                    angle_state.motor_angle = Int64_To_Pos(raw_angle);
                }
                else if(rx_data[0] == static_cast<uint8_t>(CommandType_Index::Communication_Type_READ_SINGLE_ANGLE)) //单圈角度
                {
                    angle_state.circle_angle = Int32_To_Pos(static_cast<int32_t>(
                        (rx_data[4] << 0 ) |
                        (rx_data[5] << 8 ) |
                        (rx_data[6] << 16) |
                        (rx_data[7] << 24) ));
                }
                break;
            case ReplyType::Feedback:
                //待补充
                break;
            default:
                break;
        }

        return true;
    }
} 