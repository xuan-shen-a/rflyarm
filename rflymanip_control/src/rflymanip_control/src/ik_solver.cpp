/**
  ******************************************************************************
  * @file     ik_solver.cpp
  * @author   wt
  * @date     2026/01/31
  * @brief    机械臂逆运动学求解器实现
  * @param[in] x,y,z 目标末端X,Y,Z坐标（mm）
  * @param[in] alpha_rad 末端姿态角（弧度）
  * @param[in] theta_rad 腕部旋转角度（弧度）
  * @param[in] gripper_rad 夹爪开合角度（弧度）
  * @return std::vector<double>
  *       返回关节角Joint1~Joint6（弧度）
  ******************************************************************************
 **/

#include "rflymanip_control/ik_solver.hpp"
#include <cmath>
#include <algorithm>

std::vector<double> IKSolver::solve(double x, double y, double z, double alpha_rad, double theta_rad, double gripper_rad)
{
    // 机械臂参数
    const double L5 = 154.52;
    const double D  = 23.39;
    const double L4t = 28.81;
    const double L4z = 33.01;
    const double L3 = 246;
    const double L2 = 350;

    double alpha = alpha_rad;   // 直接使用弧度
    double theta5 = theta_rad;  // 直接使用弧度
    const double delta_alpha = 0.152763096; // atan2(L5, D)

    // 逆运动学解算
    double theta1 = atan2(y, x);

    double T5 = fabs(sqrt(x*x + y*y) - sqrt(D*D + L5*L5) * cos(alpha + delta_alpha));

    double Z5 = z - sqrt(L5*L5 + D*D) * sin(alpha + delta_alpha);
    double T4 = fabs(T5 - L4t);
    double Z4 = Z5 - L4z;
    double R4 = sqrt(T4*T4 + Z4*Z4);

    double c3 = (L2*L2 + L3*L3 - R4*R4) / (2*L2*L3);
    c3 = std::clamp(c3, -1.0, 1.0); // 防止数值误差导致的越界
    double theta3 = acos(c3);       // 肘上解0～pi
    double phi = atan2(Z4, T4);     // 0～pi/2

    double c20 = (L2*L2 + R4*R4 - L3*L3) / (2*L2*R4);
    c20 = std::clamp(c20, -1.0, 1.0);
    double theta20 = acos(c20);

    double theta2 = phi + theta20;

    // 夹爪角度：直接使用输入的弧度
    double gripper_angle = gripper_rad;  // 输入为弧度，直接使用

    // 与真实值映射，添加偏移
    double delta1 = 0.0, delta2 = 0.0, delta3 = 0.0, delta4 = 0.0, delta6 = 0.0;

    double Joint1 = theta1 + delta1;        // 底座旋转
    double Joint2 = theta2 + delta2;        // 整体俯仰
    double Joint3 = theta3 + delta3;        // 肘部俯仰
    double Joint4 = alpha + delta4;         // 腕部俯仰
    double Joint5 = theta5;                 // 腕部旋转
    double Joint6 = gripper_angle + delta6; // 夹爪开合（直接使用输入角度）

    return {
        Joint1, Joint2, Joint3, Joint4, Joint5, Joint6
    };
}
