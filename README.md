# 惯导机械编排（C++）
## 1 说明
本说明以 `Arch Linux` 为例，`Eigen` 库采用系统库 `<eigen3/Eigen/Dense>`，如果没有请下载。
```Shell
sudo pacman -S eigen
```
## 2 构建
项目采用 `CMake` 构建，根据需求环境选择构建配置。
### 2.1 快速构建
使用 `Ninja` 生成。
```PowerShell
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
```
生成的可执行程序路径为 `./bin/pure_ins.exe`，其工作路径与工作目录相同。例如，将示例数据文件放在 `./data/test/` 然后解算：
```Shell
./bin/pure_ins
```
程序将读取 `./data/test/` 中 `imu.bin` 和 `ref.bin` 的数据，并在相同目录输出 `res.bin` 和 `diff.bin`。
### 2.2 代码调试（以 VSCode 为例）
> 注意：Debug 版本的可执行文件运行效率较低（解算耗时约为 Release 版本的 100 倍），如果需要快速得到结果请选择生成 Release 版本。

参见 [推荐的 VSCode 配置](https://github.com/ChuJiani/dev-config/blob/main/code/cpp.md)
 
## 3 绘图
附带一个简易的画图脚本，需要 `python` 环境及 `numpy` 和 `matplotlib`。
```PowerShell
python ./plot.py
```