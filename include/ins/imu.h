#ifndef IMU_H
#define IMU_H

#include <Eigen/Dense>

struct State {
    // State 与 Imu 严格区分，State 仅作展示数据用，
    // 迭代计算中全程使用 Imu 自身数据，避免量纲转换带来。

    // 状态值，角度单位为 deg
    double time_ = 0.0;
    double pos_[3] = {0.0};
    double vel_[3] = {0.0};
    double euler_[3] = {0.0};

    // Constructor
    State(){};
    State(double state[10]);
    State(double time, Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d euler);

    void show_state();
    State minus(State other);
};

class Imu {
  private:
    // 观测值
    double time_ = 0.0;                               // 时间戳(s)
    Eigen::Vector3d gyro_ = Eigen::Vector3d::Zero();  // 陀螺数据(角度增量, rad)
    Eigen::Vector3d acc_ = Eigen::Vector3d::Zero();   // 加表数据(速度增量, m/s)
    // 状态值
    Eigen::Vector3d euler_ = Eigen::Vector3d::Zero();  // 欧拉角(rad)
    Eigen::Vector3d vel_ = Eigen::Vector3d::Zero();    // 速度(m/s)
    Eigen::Vector3d pos_ = Eigen::Vector3d::Zero();    // 位置(rad, rad, m)

  public:
    Imu(){};
    ~Imu(){};

    // 读取观测值
    int read(FILE* fp);       // 从文件流直接读取
    int read(double buf[7]);  // 从 buf 中读取

    // 设置状态值
    void inline set_euler(const Eigen::Vector3d& euler) { euler_ = euler; }
    void inline set_vel(const Eigen::Vector3d& vel) { vel_ = vel; }
    void inline set_pos(const Eigen::Vector3d& pos) { pos_ = pos; }
    void set_state(State state);

    // 通过之前历元的观测值和状态值更新状态值
    int update(const Imu& imu_old, const Imu& imu_now);
    int update_zero_speed(const Imu& imu_old, const Imu& imu_now);    // 零速更新

    // 获取观测值
    double get_time() const { return time_; }
    Eigen::Vector3d get_gyro() const { return gyro_; }
    Eigen::Vector3d get_acc() const { return acc_; }
    // 获取状态值
    Eigen::Vector3d get_euler() const { return euler_; }
    Eigen::Vector3d get_vel() const { return vel_; }
    Eigen::Vector3d get_pos() const { return pos_; }

    // 获取整体状态
    State get_state() const { return State(time_, pos_, vel_, euler_); }
    State get_state_r2d() const;

    // 显示观测值
    int show_imu() const;
    // 显示状态值
    int show_state_r2d() const;
};

#endif