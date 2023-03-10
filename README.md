# 惯导机械编排（C++）
## 1 说明
1. 将 `eigen` 库的 `Eigen` 文件夹放入 `./include`，即可直接构建，不依赖系统环境。
2. 源码在 `Windows 10` 和 `Arch Linux` 均已通过测试，本说明以 `Windows 10` 为例。如果想使用 Linux 的系统 `eigen` 库进行构建，参见 [linux 分支](https://github.com/ChuJiani/ins/tree/linux)。
## 2 构建
项目采用 `CMake` 构建，根据需求环境选择构建配置。
### 2.1 快速构建
使用 `Ninja` 生成。
```PowerShell
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
```
生成的可执行程序路径为 `./bin/pure_ins.exe`，其工作路径与工作目录相同。例如，将示例数据文件放在 `./data/test/` 然后解算：
```PowerShell
 ./bin/pure_ins.exe test
 ```
 程序将读取 `./data/test/` 中 `imu.bin` 和 `ref.bin` 的数据，并在相同目录输出 `res.bin` 和 `diff.bin`。

 带有零速修正功能的可执行程序路径为 `./bin/pure_ins_check.exe`，执行方式相同。
 ```PowerShell
 ./bin/pure_ins_check.exe group
 ```
 程序将读取 `./data/group/` 中 `imu.bin` 和 `ref.bin` 的数据，并读取 `zero_periods.txt` 中的数值作为零速区域进行解算，解算完成后在相同目录输出 `res_check.bin` 和 `diff_check.bin`。
 ### 2.2 代码调试（以 Visual Studio 为例）
 > 注意：Debug 版本的可执行文件运行效率较低（解算耗时约为 Release 版本的 100 倍），如果需要快速得到结果请选择生成 Release 版本。

 有多种方法可以选择：
 - 在资源管理器中，根目录右键菜单选择 `Open with Visual Studio`，打开后 Visual Studio 会识别 CMake 项目并自动配置。
 - 直接生成工程文件：
    ```PowerShell
    cmake -B build -G "Visual Studio 17 2022"
    ```
    双击 `./build/pure_ins.sln` 即可用 Visual Studio 打开配置好的项目。
    > 注意：此时 Visual Studio 默认设置工作目录为 `$(ProjectDir)/` 也即 `./build/`，需要在调试配置中将其改为 `$(ProjectDir)/../` 以防打开文件出错。
 ## 3 绘图
 附带一个简易的画图脚本，需要 `python` 环境及 `numpy` 和 `matplotlib`。
 ```PowerShell
 python ./plot.py
 ```