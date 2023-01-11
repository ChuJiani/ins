// 解算时进行零速修正

#include <algorithm>
#include <cstdio>
#include <Eigen/Dense>
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
    printf("\x1b[32m[Info] Proceeding data in \"%s\".\n\x1b[0m", file_root.c_str());
    string imu_file = file_root + "imu.bin";
    string ref_file = file_root + "ref.bin";
    string res_file = file_root + "res_check.bin";
    string diff_file = file_root + "diff_check.bin";
    string period_file = file_root + "zero_periods.txt";

    // 读取零速时段，如读取失败则不进行零速修正
    printf("Loading zero periods...\n");
    vector<pair<double, double>> zero_periods;
    FILE *fp_periods = fopen(period_file.c_str(), "r");
    if (fp_periods == nullptr) {
        printf("\x1b[31m[Fatal] No \"zero_periods.txt\" detected.\x1b[0m\n");
        return -1;
    } else {
        double first, second;
        while (fscanf(fp_periods, "%lf %lf", &first, &second) == 2) {
            zero_periods.emplace_back(first, second);
        }
        if (zero_periods.size() == 0) {
            printf("\x1b[31m[Fatal] No data in \"zero_periods.txt\".\x1b[0m\n");
            return -1;
        }
    }
    fclose(fp_periods);

    // 读取参考数据
    printf("Loading ref data file...\n");
    FILE *fp_ref = fopen(ref_file.c_str(), "rb");
    if (fp_ref == nullptr) {
        printf("Error: cannot open file.\n");
        return -1;
    }
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

    // 时间对齐到参考结第 1 时刻
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

    // 解算结果容器
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

        // 判断是否在零速时段
        double time_new = imu_new.get_time();
        using pair = pair<double, double>;
        bool within_zero_period = std::any_of(zero_periods.begin(), zero_periods.end(), [time_new](pair p) {
            return time_new >= p.first && time_new < p.second;
        });
        if (within_zero_period) {
            // 零速修正
            imu_new.update_zero_speed(imu_old, imu_now);
        } else {
            // 不进行零速修正
            imu_new.update(imu_old, imu_now);
        }

        // 存储解算结果
        res.push_back(imu_new.get_state_r2d());  // 保存结果时角度单位为 deg

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

    return 0;
}