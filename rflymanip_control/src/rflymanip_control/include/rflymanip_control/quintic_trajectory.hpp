/**
  ******************************************************************************
  * @file     quintic_trajectory.hpp
  * @author   rxy
  * @date     2026/08/01
  * @brief    点到点五次轨迹插值器头文件
  ******************************************************************************
 **/

#ifndef QUINTIC_TRAJECTORY_HPP
#define QUINTIC_TRAJECTORY_HPP

#include <vector>

struct QuinticTrajectorySample
{
    std::vector<double> q;
    std::vector<double> dq;
    std::vector<double> ddq;
    std::vector<double> jerk;
};

class QuinticTrajectory
{
public:
    bool setTrajectory(const std::vector<double>& q0,
                       const std::vector<double>& qf,
                       double duration);
    QuinticTrajectorySample sample(double time) const;
    bool isFinished(double time) const;
    double getDuration() const;

private:
    std::vector<double> q0_;
    std::vector<double> delta_q_;
    double duration_ = 0.0;
};

#endif
