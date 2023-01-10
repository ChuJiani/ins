#include <Eigen/Dense>
#include <cstdio>
#include <iostream>

#include "ins/imu.h"
#include "ins/utils.h"

using namespace Eigen;

double get_rn(double lat) {
    // 计算卯酉圈半径
    return cgcs::a / sqrt(1 - cgcs::e * cgcs::e * sin(lat) * sin(lat));
}

double get_rm(double lat) {
    // 计算子午圈半径
    return cgcs::a * (1 - cgcs::e * cgcs::e) / pow(1 - cgcs::e * cgcs::e * sin(lat) * sin(lat), 1.5);
}

Vector3d get_g(double lat, double hei) {
    // 正常重力模型(GRS80)
    double g0 = 9.7803267715 * (1 + 0.0052790414 * sin(lat) * sin(lat) + 0.0000232718 * pow(sin(lat), 4));
    double g = g0 - (3.087691089e-6 - 4.397731e-9 * sin(lat) * sin(lat)) * hei + 0.721e-12 * hei * hei;
    return Vector3d(0.0, 0.0, g);
}

Vector3d get_w_ie(double lat) {
    // 计算地球自转角速度
    double n = cgcs::we * cos(lat);
    double e = 0.0;
    double d = -cgcs::we * sin(lat);
    return Vector3d(n, e, d);
}

Vector3d get_w_en(Vector3d &pos, Vector3d &vel) {
    // 大地坐标系转换到导航坐标系的角速度
    double n = vel(1) / (get_rn(pos(0)) + pos(2));
    double e = -vel(0) / (get_rm(pos(0)) + pos(2));
    double d = -vel(1) * tan(pos(0)) / (get_rn(pos(0)) + pos(2));
    return Vector3d(n, e, d);
}

Matrix3d get_skew(const Vector3d &v) {
    Matrix3d m;
    m << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
    return m;
}

Matrix3d euler_to_dcm(Vector3d &euler) {
    // 姿态角转换为姿态矩阵
    Matrix3d dcm;
    double c_phi = cos(euler(0));
    double s_phi = sin(euler(0));
    double c_theta = cos(euler(1));
    double s_theta = sin(euler(1));
    double c_psi = cos(euler(2));
    double s_psi = sin(euler(2));
    dcm << c_theta * c_psi, -c_phi * s_psi + s_phi * s_theta * c_psi, s_phi * s_psi + c_phi * s_theta * c_psi,
        c_theta * s_psi, c_phi * c_psi + s_phi * s_theta * s_psi, -s_phi * c_psi + c_phi * s_theta * s_psi, -s_theta,
        s_phi * c_theta, c_phi * c_theta;
    return dcm;
}

Vector3d dcm_to_euler(Matrix3d &dcm) {
    // 姿态矩阵转换为姿态角
    Vector3d euler;
    euler(0) = atan2(dcm(2, 1), dcm(2, 2));
    euler(1) = atan2(-dcm(2, 0), sqrt(dcm(2, 1) * dcm(2, 1) + dcm(2, 2) * dcm(2, 2)));
    euler(2) = atan2(dcm(1, 0), dcm(0, 0));
    return euler;
}

int Imu::update(const Imu &imu_old, const Imu &imu_now) {
    // 时间间隔
    double dt = time_ - imu_now.get_time();

    // 观测值提取(均为增量形式)
    Vector3d gyro_now = imu_now.get_gyro();
    Vector3d acc_now = imu_now.get_acc();
    Vector3d gyro_new = gyro_;
    Vector3d acc_new = acc_;

    // 状态值提取
    Vector3d euler_old = imu_old.get_euler();
    Vector3d vel_old = imu_old.get_vel();
    Vector3d pos_old = imu_old.get_pos();
    Vector3d euler_now = imu_now.get_euler();
    Vector3d vel_now = imu_now.get_vel();
    Vector3d pos_now = imu_now.get_pos();

    // 外推并计算 k-1/2 时刻状态值
    Vector3d vel_mid = (vel_now * 3 - vel_old) / 2;
    Vector3d pos_mid = (pos_now * 3 - pos_old) / 2;

    // 计算方向余弦矩阵
    Matrix3d dcm_now = euler_to_dcm(euler_now);

    // ------------------- 姿态更新部分 ------------------- //
    // 等效旋转矢量(b系)
    Vector3d gyro_cross = gyro_now.cross(gyro_new);
    Vector3d theta = gyro_new + gyro_cross / 12;
    // 等效旋转矩阵(b系)
    Matrix3d theta_skew = get_skew(theta);
    double theta_norm = theta.norm();
    double a1 = sin(theta_norm) / theta_norm;
    double a2 = (1 - cos(theta_norm)) / (theta_norm * theta_norm);
    Matrix3d C_bb = Matrix3d::Identity() + a1 * theta_skew + a2 * theta_skew * theta_skew;

    // 等效旋转矢量(n系)
    Vector3d w_ie = get_w_ie(pos_now(0));        // 地球自转角速度
    Vector3d w_en = get_w_en(pos_now, vel_now);  // 大地坐标系转换到导航坐标系的角速度
    Vector3d zeta = (w_ie + w_en) * dt;
    // 等效旋转矩阵(n系)
    Matrix3d zeta_skew = get_skew(zeta);
    double zeta_norm = zeta.norm();
    double b1 = sin(zeta_norm) / zeta_norm;
    double b2 = (1 - cos(zeta_norm)) / (zeta_norm * zeta_norm);
    Matrix3d C_nn = Matrix3d::Identity() - b1 * zeta_skew + b2 * zeta_skew * zeta_skew;

    // 计算新的姿态矩阵
    Matrix3d dcm_new = C_nn * dcm_now * C_bb;
    // 新的姿态角
    Vector3d euler_new = dcm_to_euler(dcm_new);

    // ------------------- 速度更新部分 ------------------- //
    // 哥氏积分项(使用 k-1/2 时刻的位置速度计算)
    Vector3d gp = get_g(pos_mid(0), pos_mid(2));     // 当地重力
    Vector3d w_ie_mid = get_w_ie(pos_mid(0));        // 地球自转角速度
    Vector3d w_en_mid = get_w_en(pos_mid, vel_mid);  // 大地坐标系转换到导航坐标系的角速度
    Vector3d acc_gn = gp - (2 * w_ie_mid + w_en_mid).cross(vel_mid);
    Vector3d delta_vel_gn = acc_gn * dt;

    // b 系比力积分项(考虑旋转效应补偿和划桨效应补偿)
    Vector3d delta_vel_fb =
        acc_new + 0.5 * gyro_new.cross(acc_new) + (gyro_now.cross(acc_new) + acc_now.cross(gyro_new)) / 12;
    // 比力积分项(使用 k-1/2 时刻的位置速度计算)
    Vector3d zeta_mid = (w_ie_mid + w_en_mid) * dt;
    Vector3d delta_vel_fn = (Matrix3d::Identity() - 0.5 * get_skew(zeta_mid)) * dcm_now * delta_vel_fb;

    // 速度更新
    Vector3d vel_new = vel_now + delta_vel_gn + delta_vel_fn;

    // ------------------- 位置更新部分 ------------------- //
    double lat_now = pos_now(0);
    double lon_now = pos_now(1);
    double hei_now = pos_now(2);
    double rm_now = get_rm(lat_now);
    double rn_now = get_rn(lat_now);
    // 新的高程
    double hei_new = hei_now - 0.5 * (vel_now(2) + vel_new(2)) * dt;
    // 新的纬度
    double hei_mid = (hei_now + hei_new) / 2;
    double lat_new = lat_now + 0.5 * (vel_now(0) + vel_new(0)) * dt / (rm_now + hei_mid);
    // 新的经度
    double lat_mid = (lat_now + lat_new) / 2;
    double rn_mid = get_rn(lat_mid);
    double lon_new = lon_now + 0.5 * (vel_now(1) + vel_new(1)) * dt / ((rn_mid + hei_mid)) * cos(lat_mid);

    // ------------------- 整体更新 ------------------- //
    this->set_euler(euler_new);
    this->set_vel(vel_new);
    this->set_pos(Vector3d(lat_new, lon_new, hei_new));

    return 0;
}