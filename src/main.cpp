#include <cstdio>
#include <eigen3/Eigen/Dense>
#include <vector>

#include "ins/imu.h"
#include "ins/utils.h"

using namespace std;
using namespace Eigen;

int main() {
    // 打开参考数据文件
    printf("Loading reference data file...\n");
    FILE *fp_ref = fopen("data/test/ref.bin", "rb");
    if (fp_ref == nullptr) {
        printf("Error: cannot open file.\n");
        return -1;
    }

    // 读取参考数据
    vector<State> ref;
    while (!feof(fp_ref)) {
        double buf[10];
        fread(buf, sizeof(double), 10, fp_ref);
        ref.push_back(State(buf));
    }
    fclose(fp_ref);

    // (弃用)
    // 第 0 时刻状态(由实验指导书给出)
    // double init_state_1[10] = {
    //    91620.0,           23.1373950708,    113.3713651222, 2.175, 0.0, 0.0,
    //    0.0, 0.0107951084511778, -2.14251290749072, -75.7498049314083};
    // 第 1 时刻状态(由参考结果给出)
    // double init_state_2[10] = {0.0};
    // fread(init_state_2, sizeof(double), 10, fp_ref);
    // (采用) 初始状态全部使用 ref 中的数据
    State init_state_1 = ref[0];
    State init_state_2 = ref[1];

    // 打开原始数据文件
    printf("Loading raw data file...\n");
    FILE *fp = fopen("data/test/imu.bin", "rb");
    if (fp == nullptr) {
        printf("Error: cannot open file.\n");
        return -1;
    }

    // 当时间到达起始时间时,进入解算循环
    double raw_buf[7];
    fread(raw_buf, sizeof(double), 7, fp);
    double current_time = raw_buf[0];
    while (current_time < init_state_1.time) {
        fread(raw_buf, sizeof(double), 7, fp);
        current_time = raw_buf[0];
    }
    // 补偿至第 0 时刻
    fseek(fp, -7 * sizeof(double), SEEK_CUR);

    // 设置第 0 时刻的 IMU
    Imu imu_old;
    imu_old.read(fp);
    imu_old.set_state(init_state_1);

    // 设置第 1 时刻的 IMU
    Imu imu_now;
    imu_now.read(fp);
    imu_now.set_state(init_state_2);

    // 解算结果
    vector<State> res;
    res.push_back(imu_old.get_state_r2d());
    res.push_back(imu_now.get_state_r2d());

    // 数据已对齐,可以开始解算
    while (true) {
        fread(raw_buf, sizeof(double), 7, fp);
        if (feof(fp)) {
            break;
        }

        // 读取新时刻观测值
        Imu imu_new;
        imu_new.read(raw_buf);

        // 用前两个历元的观测值和状态值计算新历元的状态值
        imu_new.update(imu_old, imu_now);
        res.push_back(imu_new.get_state_r2d());

        // 参考状态更新
        imu_old = imu_now;
        imu_now = imu_new;
    }
    fclose(fp);

    // 对 ref 和 res 数据作差
    vector<State> diff;
    for (int i = 0; i < ref.size() - 1; i++) {
        diff.push_back(ref[i].minus(res[i]));
    }

    // 保存差异结果
    printf("Saving diff data file...\n");
    FILE *fp_diff = fopen("data/test/diff.bin", "wb");
    for (auto &state : diff) {
        fwrite(&state, sizeof(double), 10, fp_diff);
    }
    fclose(fp_diff);

    // 保存解算结果
    printf("Saving res data file...\n");
    FILE *fp_res = fopen("data/test/res.bin", "wb");
    for (auto &state : res) {
        fwrite(&state, sizeof(double), 10, fp_res);
    }
    fclose(fp_res);

    // // 保存解算结果(文本格式)
    // FILE *fp_res_txt = fopen("data/test/res.txt", "w");
    // for (auto &state : res) {
    //     fprintf(fp_res_txt, "%.3f %.12f %.12f %.12f %.12f %.12f %.12f %.12f
    //     %.12f %.12f\n", state.time, state.pos[0],
    //             state.pos[1], state.pos[2], state.vel[0], state.vel[1],
    //             state.vel[2], state.euler[0], state.euler[1],
    //             state.euler[2]);
    // }

    return 0;
}