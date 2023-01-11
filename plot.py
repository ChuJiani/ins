import numpy as np
import matplotlib.pyplot as plt

def plot(res):
    # 绘制 位置-时间 图
    plt.figure('Position-Time')
    plt.subplot(311)
    plt.plot(res[:, 0], res[:, 1])
    plt.subplot(312)
    plt.plot(res[:, 0], res[:, 2])
    plt.subplot(313)
    plt.plot(res[:, 0], res[:, 3])

    # 绘制 速度-时间 图
    plt.figure('Velocity-Time')
    plt.subplot(311)
    plt.plot(res[:, 0], res[:, 4])
    plt.subplot(312)
    plt.plot(res[:, 0], res[:, 5])
    plt.subplot(313)
    plt.plot(res[:, 0], res[:, 6])

    # 绘制 角度-时间 图
    plt.figure('Angle-Time')
    plt.subplot(311)
    plt.plot(res[:, 0], res[:, 7])
    plt.subplot(312)
    plt.plot(res[:, 0], res[:, 8])
    plt.subplot(313)
    plt.plot(res[:, 0], res[:, 9])

    # 绘制 纬度-经度 图
    plt.figure('Lat-Lon')
    plt.plot(res[:, 1], res[:, 2])

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
        plot(ref)

    # 结果数据
    if PLOT_RES == 1:
        res = np.fromfile(res_path, dtype=np.float64)
        res = res.reshape(int(len(res)/10), 10)
        print('Plotting res...')
        plot(res)

    # 差异数据
    if PLOT_DIFF == 1:
        diff = np.fromfile(diff_path, dtype=np.float64)
        diff = diff.reshape(int(len(diff)/10), 10)
        print('Plotting diff...')
        plot(diff)

    