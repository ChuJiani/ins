#include <algorithm>
#include <cstdio>
#include <eigen3/Eigen/Dense>
#include <vector>

#include "ins/imu.h"
#include "ins/utils.h"

using namespace std;
using namespace Eigen;

int main(int argc, char *argv[]) {
    // 读取命令行参数
    if (argc < 2) {
        printf("\x1b[31m[Error] Please provide a group name as the first command line argument\x1b[0m");
        return -1;
    }
    string group_name = argv[1];

    // 文件配置
    string file_root = "data/" + group_name + "/";
    printf("\x1b[32m[Info] Processing data in \"%s\"\n\x1b[0m", file_root.c_str());
    string imu_file = file_root + "imu.bin";
    string ref_file = file_root + "ref.bin";
    string res_file = file_root + "res.bin";
    string diff_file = file_root + "diff.bin";

    // 打开参考数据文件
    printf("Loading ref data file...\n");
    FILE *fp_ref = fopen(ref_file.c_str(), "rb");
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

    // 初始状态全部使用 ref 中的数据
    State init_state_1 = ref[0];
    State init_state_2 = ref[1];

    // 打开原始数据文件
    printf("Loading raw data file...\n");
    FILE *fp = fopen(imu_file.c_str(), "rb");
    if (fp == nullptr) {
        printf("Error: cannot open file.\n");
        return -1;
    }

    // 当时间对齐到参考结第 1 时刻
    double raw_buf[7];
    fread(raw_buf, sizeof(double), 7, fp);
    double current_time = raw_buf[0];
    while (current_time < init_state_1.time_) {
        fread(raw_buf, sizeof(double), 7, fp);
        current_time = raw_buf[0];
    }
    // 补偿至第 1 时刻(前移 1 时刻)
    fseek(fp, -7 * sizeof(double), SEEK_CUR);

    // 设置第 1 时刻的 IMU
    Imu imu_old;
    imu_old.read(fp);
    imu_old.set_state(init_state_1);

    // 设置第 2 时刻的 IMU
    Imu imu_now;
    imu_now.read(fp);
    imu_now.set_state(init_state_2);

    // 解算结果
    vector<State> res;
    res.push_back(imu_old.get_state_r2d());
    res.push_back(imu_now.get_state_r2d());

    // 数据已对齐,可以开始解算
    while (true) {
        static double tmp_buf[7] = {0.0};
        fread(tmp_buf, sizeof(double), 7, fp);
        if (feof(fp)) {
            break;
        }

        // 读取新时刻观测值
        Imu imu_new;
        imu_new.read(tmp_buf);

        // 用前两个历元的观测值和状态值计算新历元的状态值
        imu_new.update(imu_old, imu_now);
        res.push_back(imu_new.get_state_r2d());  // 转换成结果需要 rad2deg

        // 参考状态更新
        imu_old = imu_now;
        imu_now = imu_new;
    }
    fclose(fp);

    // 对 ref 和 res 数据作差
    vector<State> diff;
    for (int i = 0; i < min(ref.size(), res.size()); i++) {
        diff.push_back(ref[i].minus(res[i]));
    }

    // 保存差异结果
    printf("Saving diff data file...\n");
    FILE *fp_diff = fopen(diff_file.c_str(), "wb");
    for (auto &state : diff) {
        fwrite(&state, sizeof(double), 10, fp_diff);
    }
    fclose(fp_diff);

    // 保存解算结果
    printf("Saving res data file...\n");
    FILE *fp_res = fopen(res_file.c_str(), "wb");
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