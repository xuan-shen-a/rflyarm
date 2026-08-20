/**
  ******************************************************************************
  * @file     path_planning.cpp
  * @author   wt
  * @date     2026/04/23
  * @brief    机械臂路径规划实现，使用五次多项式插值生成两状态间平滑路径
  * @param[in] x_init,y_init,z_init,alpha_init,theta_init,d_init
  *       起始末端X,Y,Z坐标（mm）、起始末端姿态角（度）、起始腕部旋转角度（度）、起始夹爪开合距离（mm）
  * @param[in] x_end,y_end,z_end,alpha_end,theta_end,d_end
  *       目标末端X,Y,Z坐标（mm）、目标末端姿态角（度）、目标腕部旋转角度（度）、目标夹爪开合距离（mm）
  * @param[in] time_duration 规划的总时间（秒）
  * @return std::vector<std::vector<double>>
  *       返回一个包含多个时间步的路径，每个时间步包含末端的X,Y,Z坐标（mm）、姿态角（度）、腕部旋转角度（度）和夹爪开合距离（mm）
  ******************************************************************************
 **/

#include <vector>
#include <algorithm>  // for std::clamp, std::max
#include <cmath>      // for std::ceil
#include <stdexcept>  // for std::invalid_argument
#include <cstddef>    // for std::size_t

namespace
{
  constexpr double kSamplePeriod = 0.02;   // 20 ms -> 50 Hz
  constexpr double kEps          = 1e-9;

  // 将角度限制到 (-180, 180]
  double wrapAngleDeg(double angle_deg)
  {
    while (angle_deg <= -180.0) angle_deg += 360.0;
    while (angle_deg >   180.0) angle_deg -= 360.0;
    return angle_deg;
  }

  // 计算从 start 到 end 的最短角度差
  double shortestAngleDeltaDeg(double start_deg, double end_deg)
  {
    return wrapAngleDeg(end_deg - start_deg);
  }

  // 五次多项式时间缩放因子 s(t)
  // tau = t / T, tau ∈ [0, 1]
  // s(t) = 10*tau^3 - 15*tau^4 + 6*tau^5
  double quinticBlend(double t, double T)
  {
    if (T <= kEps)
    {
      return 1.0;
    }

    double tau = t / T;
    tau = std::clamp(tau, 0.0, 1.0);

    return 10.0 * tau * tau * tau - 15.0 * tau * tau * tau * tau +  6.0 * tau * tau * tau * tau * tau;
  }

  // 普通标量五次多项式插值
  double quinticInterpolate(double start, double end, double t, double T)
  {
    double s = quinticBlend(t, T);
    return start + (end - start) * s;
  }

  // 角度五次多项式插值（按最短路径）
  double quinticInterpolateAngleDeg(double start_deg, double end_deg, double t, double T)
  {
    double s = quinticBlend(t, T);
    double delta = shortestAngleDeltaDeg(start_deg, end_deg);
    return wrapAngleDeg(start_deg + delta * s);
  }
}

/**
 * @brief 生成机械臂末端路径
 */
std::vector<std::vector<double>> planArmPath(
    double x_init,     double y_init,     double z_init,
    double alpha_init, double theta_init, double d_init,
    double x_end,      double y_end,      double z_end,
    double alpha_end,  double theta_end,  double d_end,
    double time_duration)
{
    if (time_duration < 0.0)
    {
        throw std::invalid_argument("time_duration must be non-negative.");
    }

    std::vector<std::vector<double>> path;

    // 若总时间过小，则直接返回终点状态
    if (time_duration <= kEps)
    {
        path.push_back({
            x_end,
            y_end,
            z_end,
            wrapAngleDeg(alpha_end),
            wrapAngleDeg(theta_end),
            d_end
        });
        return path;
    }

    // 保证至少有起点和终点两个点
    const std::size_t point_num =
        std::max<std::size_t>(2, static_cast<std::size_t>(std::ceil(time_duration / kSamplePeriod)) + 1);

    path.reserve(point_num);

    for (std::size_t i = 0; i < point_num; ++i)
    {
        double t = static_cast<double>(i) * time_duration / static_cast<double>(point_num - 1);

        double x     = quinticInterpolate(x_init,     x_end,     t, time_duration);
        double y     = quinticInterpolate(y_init,     y_end,     t, time_duration);
        double z     = quinticInterpolate(z_init,     z_end,     t, time_duration);
        double alpha = quinticInterpolateAngleDeg(alpha_init, alpha_end, t, time_duration);
        double theta = quinticInterpolateAngleDeg(theta_init, theta_end, t, time_duration);
        double d     = quinticInterpolate(d_init,     d_end,     t, time_duration);

        path.push_back({x, y, z, alpha, theta, d});
    }

    return path;
}
