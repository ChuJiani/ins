import numpy as np

# 文件配置
root = 'data/group/'
raw = root + 'ref.txt'
out = root + 'ref.bin'

# 读取文本文件，输出二进制文件
data = np.loadtxt(raw, dtype=np.float64)
data.tofile(out)

# 尝试读取输出结果，验证程序正确性
data = np.fromfile(out, dtype=np.float64)
data = data.reshape(int(len(data)/10), 10)
print(data.shape)