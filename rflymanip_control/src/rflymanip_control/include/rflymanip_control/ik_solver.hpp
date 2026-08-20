/**
  ******************************************************************************
  * @file     ik_solver.hpp
  * @author   wt
  * @date     2026/01/31
  * @brief    机械臂逆运动学求解器头文件
  ******************************************************************************
 **/

#ifndef IK_SOLVER_HPP
#define IK_SOLVER_HPP

#pragma once
#include <vector>

class IKSolver
{
public:
    std::vector<double> solve(
        double x,
        double y,
        double z,
        double alpha_rad,
        double theta_rad,
        double gripper_rad
    );
};

#endif
