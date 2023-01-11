import numpy as np
import matplotlib.pyplot as plt

def plot(res, head: str):
    # 绘制 位置-时间 图
    fig = plt.figure(head +"_position_time")
    axs = fig.subplots(3, 1)
    fig.suptitle('Position-Time')
    axs[0].plot(res[:, 0], res[:, 1])
    axs[0].set_xlabel('Time(s)')
    axs[0].set_ylabel('Latitude(deg)')
    axs[1].plot(res[:, 0], res[:, 2])
    axs[1].set_xlabel('Time(s)')
    axs[1].set_ylabel('Longitude(deg)')
    axs[2].plot(res[:, 0], res[:, 3])
    axs[2].set_xlabel('Time(s)')
    axs[2].set_ylabel('Altitude(m)')
    fig.tight_layout()

    # 绘制 速度-时间 图
    fig = plt.figure(head + "_velocity_time")
    axs = fig.subplots(3, 1)
    fig.suptitle('Velocity-Time')
    axs[0].plot(res[:, 0], res[:, 4])
    axs[0].set_xlabel('Time(s)')
    axs[0].set_ylabel('North Speed(m/s)')
    axs[1].plot(res[:, 0], res[:, 5])
    axs[1].set_xlabel('Time(s)')
    axs[1].set_ylabel('East Speed(m/s)')
    axs[2].plot(res[:, 0], res[:, 6])
    axs[2].set_xlabel('Time(s)')
    axs[2].set_ylabel('Down Speed(m/s)')
    fig.tight_layout()

    # 绘制 姿态-时间 图
    fig = plt.figure(head + "_attitude_time")
    axs = fig.subplots(3, 1)
    fig.suptitle('Attitude-Time')
    axs[0].plot(res[:, 0], res[:, 7])
    axs[0].set_xlabel('Time(s)')
    axs[0].set_ylabel('Roll(deg)')
    axs[1].plot(res[:, 0], res[:, 8])
    axs[1].set_xlabel('Time(s)')
    axs[1].set_ylabel('Pitch(deg)')
    axs[2].plot(res[:, 0], res[:, 9])
    axs[2].set_xlabel('Time(s)')
    axs[2].set_ylabel('Heading(deg)')
    fig.tight_layout()

    # 绘制 纬度-经度 图
    fig = plt.figure(head + "_latitude_longitude")
    axs = fig.subplots(1, 1)
    fig.suptitle('Latitude-Longitude')
    axs.plot(res[:, 1], res[:, 2])
    axs.set_xlabel('Latitude(deg)')
    axs.set_ylabel('Longitude(deg)')
    fig.tight_layout()

    plt.show()


if __name__ == '__main__':
    from sys import argv, exit
    try:
        # 获取文件夹名
        group_name = argv[1]
        group_name = str(group_name)

    except IndexError:
        # 参数异常
        print("\x1b[31m[Error] Please provide a group name as the first command line argument\x1b[0m")
        exit()

    # 文件配置
    data_root = 'data/' + str(argv[1]) + '/'
    ref_path = data_root + 'ref.bin'
    res_path = data_root + 'res.bin'
    diff_path = data_root + 'diff.bin'

    # 画图配置
    PLOT_REF = 1
    PLOT_RES = 1
    PLOT_DIFF = 1

    # 参考数据
    if PLOT_REF == 1:
        ref = np.fromfile(ref_path, dtype=np.float64)
        ref = ref.reshape(int(len(ref)/10), 10)
        print('Plotting ref...')
        plot(ref, "ref")

    # 结果数据
    if PLOT_RES == 1:
        res = np.fromfile(res_path, dtype=np.float64)
        res = res.reshape(int(len(res)/10), 10)
        print('Plotting res...')
        plot(res, "res")

    # 差异数据
    if PLOT_DIFF == 1:
        diff = np.fromfile(diff_path, dtype=np.float64)
        diff = diff.reshape(int(len(diff)/10), 10)
        print('Plotting diff...')
        plot(diff, "diff")

    