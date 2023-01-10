#ifndef UTILS_H
#define UTILS_H

constexpr double pi = 3.14159265358979323846;
constexpr double RAD_TO_DEG = 180.0 / pi;  // 弧度转角度
constexpr double DEG_TO_RAD = pi / 180.0;  // 角度转弧度
constexpr double DELTA_T = 0.005;          // 采样间隔(s)

namespace cgcs {
constexpr static double we = 7.292115e-5;   // 地球自转角速度
constexpr static double a = 6378137.0;      // 长半轴
constexpr static double e = 0.08181919104;  // 第一偏心率
};                                          // namespace cgcs

#endif