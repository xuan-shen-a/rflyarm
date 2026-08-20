/**
  ******************************************************************************
  * @file     lkmotor_driver.hpp
  * @author   rxy
  * @date     2026/01/28
  * @brief    LK关节电机驱动结构及接口
  ******************************************************************************
  * @note     LK关节电机通讯协议：
              主控向总线发送单电机命令，对应 ID 的电机在收到命令后执行，并向主控发送回复
              1.标识符：
              命令报文标识符：0x140 + ID(1~32)
              回复报文标识符：0x180 + ID(1~32)
              2.数据帧：0x
              帧格式：数据帧
              帧类型：标准帧
              DLC：8字节 ( DATA[0]命令字节 + DATA[1]~DATA[7]数据域 )
              3.回复报文类型：
              (1) 电机状态1：MotorTotalState
                0x9A : 读取电机状态1
                0x9B : 清楚错误标志
              (2) 电机状态2：MotorWorkState
                0x9C : 读取电机状态2
                0xA1 - 0xA8 : 电流、速度、位置控制命令
              (3) 电机状态3：MotorCurrentState
                0x9D : 读取电机状态3
              (4) 电机控制参数：MotorParamState
                0xC0 : 读取控制参数
                0xC1 : 写入控制参数
              (5) 编码器状态：MotorEncoderState
                0x90 : 读取编码器数据
                0x19 : 设置电机零点(ROM)
              (6) 多圈角度：MotorAngleState
                0x92 : 读取多圈角度
                0x94 : 读取单圈角度
              (7) 返回与发送帧相同
                0x80 : 失能电机
                0x81 : 停止电机
                0x88 : 使能电机
                0x95 : 设置电机位置(RAM)

              ps. 更改该文件需严格对照LK通讯协议，谨慎更改
 ****************************************************************************
 **/

#ifndef LKMOTOR_DRIVER_HPP
#define LKMOTOR_DRIVER_HPP

#include <cstdint>
#include <cstring>
#include <cmath>
#include "can_msgs/msg/frame.hpp"

namespace lkmotor
{
/* -------------------------------------------------- 参数列表 ----------------------------------------------- */
    enum class CommandType_Index : uint8_t 
    {
        Communication_Type_READ_STATUS_1       = 0x9A, // 温度, 电压, 电流, 状态, 错误
        Communication_Type_CLEAR_ERROR         = 0x9B, // 清除错误
        Communication_Type_READ_STATUS_2       = 0x9C, // 温度, 力矩电流, 转速, 编码器
        Communication_Type_READ_STATUS_3       = 0x9D, // 温度, A/B/C 相电流
        Communication_Type_READ_ENCODER        = 0x90, // 编码器原始值, 零偏
        Communication_Type_READ_MULTI_ANGLE    = 0x92, // 多圈角度 (int64)
        Communication_Type_READ_SINGLE_ANGLE   = 0x94, // 单圈角度 (0-360)

        Communication_Type_MOTOR_OFF           = 0x80, // 失能电机
        Communication_Type_MOTOR_STOP          = 0x81, // 停止电机
        Communication_Type_MOTOR_RUN           = 0x88, // 使能电机
        Communication_Type_BRAKE_OPEN          = 0x8C, // 抱闸释放 (0x01) / 锁死 (0x00)

        Communication_Type_CUR_LOOP            = 0xA1, // 转矩闭环 (Iq控制)
        Communication_Type_VEL_LOOP            = 0xA2, // 速度闭环 (带转矩限制)
        Communication_Type_POS_MULTI_LOOP_1    = 0xA3, // 多圈位置闭环 1 (不带限速)
        Communication_Type_POS_MULTI_LOOP_2    = 0xA4, // 多圈位置闭环 2 (带限速)
        Communication_Type_POS_SINGLE_LOOP_1   = 0xA5, // 单圈位置闭环 1 (方向 + 角度)
        Communication_Type_POS_SINGLE_LOOP_2   = 0xA6, // 单圈位置闭环 2 (方向 + 限速 + 角度)
        Communication_Type_POS_INC_LOOP_1      = 0xA7, // 增量位置闭环 1
        Communication_Type_POS_INC_LOOP_2      = 0xA8, // 增量位置闭环 2 (带限速)

        Communication_Type_READ_PARAM          = 0xC0, // 读取控制参数
        Communication_Type_WRITE_PARAM         = 0xC1, // 写入控制参数
        Communication_Type_SET_ZERO_ROM        = 0x19, // 设置电机零点 (ROM)
        Communication_Type_SET_POS_RAM         = 0x95  // 设置电机位置 (RAM)
    };

    enum class ControlMode_Index : uint8_t      // 位置控制模式中，2模式带限速参数
    {                           
         Control_Mode_Cur                      = 0x01, // 电流控制
         Control_Mode_Vel                      = 0x02, // 速度控制
         Control_Mode_MultiLoopPos1            = 0x03, // 多圈位置控制1
         Control_Mode_MultiLoopPos2            = 0x04, // 多圈位置控制2
         Control_Mode_SingleLoopPos1           = 0x05, // 单圈位置控制1
         Control_Mode_SingleLoopPos2           = 0x06, // 单圈位置控制2
         Control_Mode_IncrementalPos1          = 0x07, // 增量位置控制1
         Control_Mode_IncrementalPos2          = 0x08  // 增量位置控制2
    };

    enum class Param_Index : uint8_t             
    {                           
         Param_PosLoop_PID                     = 0x0A, // 位置环PID
         Param_VelLoop_PID                     = 0x0B, // 速度环PID
         Param_CurLoop_PID                     = 0x0C, // 电流环PID
         Param_Cur_Limit                       = 0x1E, // 电流Limit
         Param_Vel_Limit                       = 0x20, // 速度Limit
         Param_Pos_Limit                       = 0x22, // 位置Limit
         Param_Cur_Ramp                        = 0x24, // 电流Ramp
         Param_Vel_Ramp                        = 0x26  // 速度Ramp
    };

/* -------------------------------------------------- 信息内容 ----------------------------------------------- */
    struct MotorTotalState        // 电机状态1
    {  
        int8_t temperature;
        float voltage;
        float current;
        uint8_t motor_state;
        uint8_t error_state;
    };

    struct MotorWorkState         // 电机状态2
    {
        int8_t temperature;
        float iq;
        float speed;
        uint16_t encoder;
    };

    struct MotorCurrentState      // 电机状态3
    {
        int8_t temperature;
        float current_a;
        float current_b;
        float current_c;
    };

    struct MotorParamState        // 电机控制参数
    {
        uint16_t pos_pid_kp;
        uint16_t pos_pid_ki;
        uint16_t pos_pid_kd;
        uint16_t vel_pid_kp;
        uint16_t vel_pid_ki;
        uint16_t vel_pid_kd;
        uint16_t cur_pid_kp;
        uint16_t cur_pid_ki;
        uint16_t cur_pid_kd;
        int16_t input_cur_limit;
        int32_t input_vel_limit;
        int32_t input_pos_limit;
        int32_t input_cur_ramp;
        int32_t input_vel_ramp;
    };

    struct MotorEncoderState      // 电机编码器原始值, 零偏
    {
        uint16_t encoder;
        uint16_t encoder_raw;       
        uint16_t encoder_offset;    
    };

    struct MotorAngleState      // 电机单圈/多圈角度  0.01°/LSB
    {
        float motor_angle;       
        float circle_angle;    
    };

/* -------------------------------------------------- 电机结构 ----------------------------------------------- */    // 单个电机对象
    class LKMotor {
    public:
        // --- 构造函数 ---
        LKMotor(uint8_t motor_id, uint8_t ratio);

        // --- 访问接口 ---
        uint8_t GetID() const { return can_id; }
        uint8_t GetRatio() const { return ratio; }
        const MotorTotalState& GetTotalState() const { return total_state; }
        const MotorWorkState& GetWorkState() const { return work_state; }
        const MotorCurrentState& GetCurrentState() const { return current_state; }
        const MotorParamState& GetParamState() const { return param_state; }
        const MotorEncoderState& GetEncoderState() const { return encoder_state; }
        const MotorAngleState& GetAngleState() const { return angle_state; }
        ControlMode_Index GetControlMode() const { return control_mode; }
        
        // --- ID 计算 ---
        uint32_t Get_ID_ToCmd() const { return 0x140 + can_id; }
        uint32_t Get_ID_FromReply(uint32_t Reply_id) const { return Reply_id - 0x180; }
 
        // --- 打包函数 ---
        can_msgs::msg::Frame CmdControlEnable();
        can_msgs::msg::Frame CmdControlDisable();
        can_msgs::msg::Frame CmdControlStop();
        can_msgs::msg::Frame CmdControlBrake(bool release);
        can_msgs::msg::Frame CmdControlSetzero();
        can_msgs::msg::Frame CmdControlCur(float current); 
        can_msgs::msg::Frame CmdControlVel(float dps, float current_limit); 
        can_msgs::msg::Frame CmdControlPosMulti1(float deg); 
        can_msgs::msg::Frame CmdControlPosMulti2(float deg, float max_dps); 
        can_msgs::msg::Frame CmdReadMultiAngle();  
        can_msgs::msg::Frame CmdReadSingleAngle(); 

        // --- 解包函数(待完善) ---
        bool ReplyUnpack(const uint8_t* rx_data);

    private:
        // --- 电机参数 ---
        uint8_t can_id; 
        uint8_t ratio;
        ControlMode_Index control_mode;  
        
        // --- 各类状态缓存 ---
        MotorTotalState total_state;
        MotorWorkState work_state;
        MotorCurrentState current_state;
        MotorParamState param_state;
        MotorEncoderState encoder_state;
        MotorAngleState angle_state;

        // --- 字节拆分函数 ---
        static void WriteInt16(uint8_t* buf, int16_t val)
        {
            buf[0] = val & 0xFF;
            buf[1] = (val >> 8) & 0xFF;
        }
        static void WriteInt32(uint8_t* buf, int32_t val)
        {
            buf[0] = val & 0xFF;
            buf[1] = (val >> 8) & 0xFF;
            buf[2] = (val >> 16) & 0xFF;
            buf[3] = (val >> 24) & 0xFF;
        }

        // --- 数值转换辅助函数  ---
        /** @brief 电流转换函数 -33A -> -2048, 33A -> 2048 **/
        int16_t Cur_To_Int16(float current) const {return static_cast<int16_t>((current / 33.0f) * 2048.0f);} 
        float Int16_To_Cur(int16_t val) const {return static_cast<float>(val * 33.0f / 2048.0f);} 

        /** @brief 速度转换函数 0.01dps/LSB **/
        int32_t Vel_To_Int32(float dps) const { return static_cast<int32_t>(dps * 100.0f); }
        float Int32_To_Vel(int32_t val) const {return static_cast<float>(val / 100.0f);} 

        /** @brief 速度转换函数 1dps/LSB **/
        int16_t Vel_To_Int16(float dps) const {return static_cast<int16_t>(dps);} 
        float Int16_To_Vel(int16_t val) const {return static_cast<float>(val);} 

        /** @brief 位置转换函数 0.01deg/LSB **/
        int32_t Pos_To_Int32(float deg) const {return static_cast<int32_t>(deg * 100.0f);} 
        float Int32_To_Pos(int32_t val) const {return static_cast<float>(val / 100.0f);} 

        int64_t Pos_To_Int64(float deg) const {return static_cast<int64_t>(deg * 100.0f);} 
        float Int64_To_Pos(int64_t val) const {return static_cast<float>(val / 100.0f);} 

        /** @brief 速度限制转换函数 1dps/LSB **/
        uint16_t VelLim_To_Uint16(float dps) const {return static_cast<uint16_t>(std::abs(dps));} 
    };

} 

#endif