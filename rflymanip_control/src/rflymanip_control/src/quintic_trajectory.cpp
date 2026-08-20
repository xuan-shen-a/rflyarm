/**
  ******************************************************************************
  * @file     quintic_trajectory.cpp
  * @author   rxy
  * @date     2026/08/01
  * @brief    点到点五次轨迹插值器实现
  ******************************************************************************
 **/

#include "rflymanip_control/quintic_trajectory.hpp"

#include <algorithm>

/** @brief 设置静止起止的点到点五次轨迹 **/
bool QuinticTrajectory::setTrajectory(
    const std::vector<double>& q0,
    const std::vector<double>& qf,
    double duration)
{
    if (q0.empty() || q0.size() != qf.size() || duration <= 0.0)
    {
        return false;
    }

    q0_ = q0;
    delta_q_.resize(q0.size());
    for (size_t i = 0; i < q0.size(); ++i)
    {
        delta_q_[i] = qf[i] - q0[i];
    }

    duration_ = duration;
    return true;
}

/** @brief 采样五次轨迹的位置、速度、加速度和jerk **/
QuinticTrajectorySample QuinticTrajectory::sample(double time) const
{
    QuinticTrajectorySample result;
    result.q.resize(q0_.size());
    result.dq.resize(q0_.size());
    result.ddq.resize(q0_.size());
    result.jerk.resize(q0_.size());

    if (duration_ <= 0.0) return result;

    double clamped_time = std::clamp(time, 0.0, duration_);
    double s = clamped_time / duration_;
    double s2 = s * s;
    double s3 = s2 * s;
    double s4 = s3 * s;
    double s5 = s4 * s;

    double position_factor = 10.0 * s3 - 15.0 * s4 + 6.0 * s5;
    double velocity_factor = (30.0 * s2 - 60.0 * s3 + 30.0 * s4) / duration_;
    double acceleration_factor = (60.0 * s - 180.0 * s2 + 120.0 * s3) /
                                 (duration_ * duration_);
    double jerk_factor = (60.0 - 360.0 * s + 360.0 * s2) /
                         (duration_ * duration_ * duration_);

    for (size_t i = 0; i < q0_.size(); ++i)
    {
        result.q[i] = q0_[i] + delta_q_[i] * position_factor;
        result.dq[i] = delta_q_[i] * velocity_factor;
        result.ddq[i] = delta_q_[i] * acceleration_factor;
        result.jerk[i] = delta_q_[i] * jerk_factor;
    }

    return result;
}

/** @brief 判断轨迹是否完成 **/
bool QuinticTrajectory::isFinished(double time) const
{
    return duration_ > 0.0 && time >= duration_;
}

/** @brief 获取轨迹时长 **/
double QuinticTrajectory::getDuration() const
{
    return duration_;
}
