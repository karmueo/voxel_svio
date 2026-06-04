# Repository Guidelines

## 项目结构与模块组织

本仓库是 ROS catkin 包，用于 Voxel-SVIO 双目视觉惯性里程计。核心 C++ 实现在 `src/`，对应头文件在 `include/`。运行参数位于 `config/`，ROS 启动文件位于 `launch/`，RViz 配置位于 `rviz_cfg/`。`doc/` 保存 README 使用的演示素材。`thirdLibrary/` 为第三方依赖源码，除非明确升级依赖，否则不要改动。

## 构建、测试与本地运行命令

请在 catkin 工作空间根目录构建，而不是在本包目录内直接构建：

```bash
cd ~/Voxel-SVIO
catkin_make
source devel/setup.bash
```

按数据集启动节点：

```bash
roslaunch voxel_svio vio_euroc.launch
roslaunch voxel_svio vio_tum_vi.launch
roslaunch voxel_svio vio_kaist.launch
```

另开终端播放 rosbag，并启用仿真时间：

```bash
rosbag play SEQUENCE_NAME.bag --clock -d 1.0
```

运行前创建 `output/` 目录；节点会在其中写入 `pose.txt`。

## 编码风格与命名约定

项目使用 C++14，并依赖 ROS、Eigen、OpenCV、PCL 和 Ceres。保持现有代码风格：4 空格缩进，函数和类的大括号独占一行，函数名使用 camelCase，例如 `readParameters`。变量命名遵循邻近代码的 lower-camel 或 snake_case。新增模块应保持头文件和实现文件成对，例如 `include/featureTracker.h` 与 `src/featureTracker.cpp`。

## 测试指南

当前 `CMakeLists.txt` 和 `package.xml` 未定义包级自动化测试。提交改动前至少运行 `catkin_make`，并针对受影响的 launch/config 路径做短 rosbag 冒烟测试。若新增测试，放在 `test/` 下，文件名对应被测模块，并接入 CMake，使 `catkin_make run_tests` 可执行。

## 提交与 Pull Request 规范

近期提交历史使用简短祈使句，例如 `Update stereoVio.cpp`；新提交应保持聚焦并说明具体对象。提交信息必须使用中文，并遵循标准的 Conventional Commits 规范，例如 `feat: 添加特征跟踪配置`、`fix: 修复位姿输出路径`。PR 需要描述算法或配置变化，列出验证过的数据集/序列，附上构建和冒烟测试结果，并说明是否影响参数或 launch 兼容性。不要提交 rosbag、`output/` 生成物、构建产物或大型本地实验数据。

## 配置与运行注意事项

不同数据集需使用匹配配置。TUM-VI 按 corridor、magistrale、room 选择 YAML。KAIST 的 `urban38` 和 `urban39` 使用 `kaist2.yaml`，其他序列使用 `kaist.yaml`。大型 KAIST 运行建议关闭或限制 RViz，因为点云和 voxel 可视化会快速增加内存占用。

## 智能体说明

面向本仓库的所有后续回答、解释、代码审查意见和操作总结均必须使用中文，除非用户明确要求使用其他语言。命令、路径、代码标识符和上游英文专有名词可保留原文。
