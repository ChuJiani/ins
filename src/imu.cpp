#include "ins/imu.h"

#include <cstdio>

#include "ins/utils.h"

State::State(double state[10]) { memcpy(this, state, sizeof(double) * 10); }

State::State(double time, Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d euler) {
    this->time = time;
    this->pos[0] = pos(0);
    this->pos[1] = pos(1);
    this->pos[2] = pos(2);
    this->vel[0] = vel(0);
    this->vel[1] = vel(1);
    this->vel[2] = vel(2);
    this->euler[0] = euler(0);
    this->euler[1] = euler(1);
    this->euler[2] = euler(2);
}

void State::show_state() {
    // 显示状态值, 角度单位为 deg
    printf("[State] %.3f\n", this->time);
    printf("  pos  : ");
    printf("%.8f %.8f %.8f\n", this->pos[0], this->pos[1], this->pos[2]);
    printf("  vel  : ");
    printf("%.8f %.8f %.8f\n", this->vel[0], this->vel[1], this->vel[2]);
    printf("  euler: ");
    printf("%.8f %.8f %.8f\n", this->euler[0], this->euler[1], this->euler[2]);
}

State State::minus(State other) {
    // 计算两个状态的差值
    State diff;
    diff.time = this->time;
    for (int i = 0; i < 3; i++) {
        diff.pos[i] = this->pos[i] - other.pos[i];
        diff.vel[i] = this->vel[i] - other.vel[i];
        diff.euler[i] = this->euler[i] - other.euler[i];
    }
    return diff;
}

int Imu::read(FILE *fp) {
    if (feof(fp)) {
        return 1;  // 文件结束
    }

    static double buf[7];
    fread(buf, sizeof(double), 7, fp);

    time_ = buf[0];
    gyro_ = Eigen::Map<Eigen::Vector3d>(buf + 1);
    acc_ = Eigen::Map<Eigen::Vector3d>(buf + 4);
    return 0;
}

int Imu::read(double buf[7]) {
    time_ = buf[0];
    gyro_ = Eigen::Map<Eigen::Vector3d>(buf + 1);
    acc_ = Eigen::Map<Eigen::Vector3d>(buf + 4);
    return 0;
}

void Imu::set_state(State state) {
    this->set_pos(Eigen::Vector3d(state.pos[0] * DEG_TO_RAD, state.pos[1] * DEG_TO_RAD, state.pos[2]));
    this->set_vel(Eigen::Vector3d(state.vel[0], state.vel[1], state.vel[2]));
    this->set_euler(Eigen::Vector3d(state.euler[0], state.euler[1], state.euler[2]) * DEG_TO_RAD);
}

int Imu::show_imu() const {
    printf("[Imu] %.3f \n", time_);
    printf("  gyro: ");
    printf("%.8f %.8f %.8f\n", gyro_(0), gyro_(1), gyro_(2));
    printf("  acc : ");
    printf("%.8f %.8f %.8f\n", acc_(0), acc_(1), acc_(2));
    return 0;
}

int Imu::show_state_r2d() const {
    // 显示状态值, 角度单位为 deg
    printf("[State] %.3f\n", time_);
    printf("  pos  : ");
    printf("%.8f %.8f %.8f\n", pos_(0) * RAD_TO_DEG, pos_(1) * RAD_TO_DEG, pos_(2));
    printf("  vel  : ");
    printf("%.8f %.8f %.8f\n", vel_(0), vel_(1), vel_(2));
    printf("  euler: ");
    printf("%.8f %.8f %.8f\n", euler_(0) * RAD_TO_DEG, euler_(1) * RAD_TO_DEG, euler_(2) * RAD_TO_DEG);
    return 0;
}

State Imu::get_state_r2d() const {
    // 获取状态值, 角度单位为 deg
    State state;
    state.time = time_;
    state.pos[0] = pos_(0) * RAD_TO_DEG;
    state.pos[1] = pos_(1) * RAD_TO_DEG;
    state.pos[2] = pos_(2);
    state.vel[0] = vel_(0);
    state.vel[1] = vel_(1);
    state.vel[2] = vel_(2);
    state.euler[0] = euler_(0) * RAD_TO_DEG;
    state.euler[1] = euler_(1) * RAD_TO_DEG;
    state.euler[2] = euler_(2) * RAD_TO_DEG;
    return state;
}