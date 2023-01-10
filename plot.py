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
    # 文件配置
    data_root = 'data/test/'
    ref_path = data_root + 'ref.bin'
    res_path = data_root + 'res.bin'
    diff_path = data_root + 'diff.bin'

    # 参考数据
    ref = np.fromfile(ref_path, dtype=np.float64)
    ref = ref.reshape(int(len(ref)/10), 10)
    print('Plotting ref...')
    plot(ref)

    # 结果数据
    res = np.fromfile(res_path, dtype=np.float64)
    res = res.reshape(int(len(res)/10), 10)
    print('Plotting res...')
    plot(res)

    # 差异数据
    diff = np.fromfile(diff_path, dtype=np.float64)
    diff = diff.reshape(int(len(diff)/10), 10)
    print('Plotting diff...')
    plot(diff)

    