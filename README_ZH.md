# Voxel-SVIO ROS2 EuRoC Vicon Room 运行指南

本文档说明如何在 ROS2 Humble/Jazzy 下运行 Voxel-SVIO，使用本机
`/mnt/data/slam/vicon_room1_ros2` 中的 EuRoC Vicon Room ROS2 bag，并在 RViz2
叠加显示估计轨迹和 Vicon 真值轨迹。

## 1. 数据集与真值

ROS2 bag 默认路径：

- `/mnt/data/slam/vicon_room1_ros2/V1_01_easy`
- `/mnt/data/slam/vicon_room1_ros2/V1_02_medium`
- `/mnt/data/slam/vicon_room1_ros2/V1_03_difficult`

节点使用的话题：

- `/cam0/image_raw`
- `/cam1/image_raw`
- `/imu0`

真值默认读取 OpenVINS 已准备好的文本文件：

- `/home/scl/work/slam/open_vins/ov_data/euroc_mav/V1_01_easy.txt`
- `/home/scl/work/slam/open_vins/ov_data/euroc_mav/V1_02_medium.txt`
- `/home/scl/work/slam/open_vins/ov_data/euroc_mav/V1_03_difficult.txt`

## 2. 构建

建议使用干净 shell 环境，避免 conda 或 `/usr/local` 下的库影响 ROS2/CMake：

```bash
cd /home/scl/work/slam/voxel_svio

export PATH=/usr/sbin:/usr/bin:/sbin:/bin
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER
unset LD_LIBRARY_PATH LIBRARY_PATH PYTHONPATH
unset CMAKE_PREFIX_PATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH

source /opt/ros/jazzy/setup.bash
# Humble 环境使用：
# source /opt/ros/humble/setup.bash

colcon build --packages-select voxel_svio --cmake-clean-cache \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

source install/setup.bash
```

确认可执行文件：

```bash
ros2 pkg executables voxel_svio
```

应包含：

```text
voxel_svio vio_node
voxel_svio error_singlerun
voxel_svio publish_groundtruth_path.py
voxel_svio evaluate_euroc_trajectory.py
```

## 3. 无头短测

不启动 RViz2，自动播放指定数据集。`dataset` 可改为 `V1_01_easy`、`V1_02_medium`
或 `V1_03_difficult`，`output_path` 建议同步改成对应目录，方便区分不同序列输出：

```bash
ros2 launch voxel_svio vio_euroc_ros2.launch.py \
  dataset:=V1_01_easy \
  rviz:=false \
  play_bag:=true \
  output_path:=/tmp/voxel_svio_v1_01
```

正常运行时，终端会输出 `q_GtoI`、`p_IinG`、`dist` 等状态。估计轨迹会写入：

```text
/tmp/voxel_svio_v1_01/pose.txt
```

如果需要拆分执行，先在终端 1 启动 Voxel-SVIO 节点和真值轨迹发布节点：

```bash
ros2 launch voxel_svio vio_euroc_ros2.launch.py \
  dataset:=V1_01_easy \
  rviz:=false \
  play_bag:=false \
  output_path:=/tmp/voxel_svio_v1_01
```

再在终端 2 单独播放 ROS2 bag：

```bash
ros2 bag play /mnt/data/slam/vicon_room1_ros2/V1_01_easy \
  --clock \
  --topics /cam0/image_raw /cam1/image_raw /imu0
```

例如切换到 `V1_02_medium`：

```bash
ros2 launch voxel_svio vio_euroc_ros2.launch.py \
  dataset:=V1_02_medium \
  rviz:=false \
  play_bag:=true \
  output_path:=/tmp/voxel_svio_v1_02
```

## 4. RViz2 可视化

有桌面环境时启用 RViz2。`dataset` 同样可以替换为 `V1_01_easy`、
`V1_02_medium` 或 `V1_03_difficult`：

```bash
ros2 launch voxel_svio vio_euroc_ros2.launch.py \
  dataset:=V1_01_easy \
  rviz:=true \
  play_bag:=true \
  output_path:=/tmp/voxel_svio_v1_01
```

例如可视化 `V1_03_difficult`：

```bash
ros2 launch voxel_svio vio_euroc_ros2.launch.py \
  dataset:=V1_03_difficult \
  rviz:=true \
  play_bag:=true \
  output_path:=/tmp/voxel_svio_v1_03
```

如果需要拆分执行，先在终端 1 启动 Voxel-SVIO 节点和真值轨迹发布节点：

```bash
ros2 launch voxel_svio vio_euroc_ros2.launch.py \
  dataset:=V1_01_easy \
  rviz:=false \
  play_bag:=false \
  output_path:=/tmp/voxel_svio_v1_01
```

再在终端 2 单独播放 ROS2 bag：

```bash
ros2 bag play /mnt/data/slam/vicon_room1_ros2/V1_01_easy \
  --clock \
  --topics /cam0/image_raw /cam1/image_raw /imu0
```

最后在终端 3 单独打开 RViz2：

```bash
rviz2 -d "$(ros2 pkg prefix voxel_svio)/share/voxel_svio/rviz_cfg/euroc_ros2.rviz" \
  --ros-args -p use_sim_time:=true
```

RViz2 固定坐标系为 `camera_init`，主要显示：

- `/vio/path`：Voxel-SVIO 估计轨迹。
- `/vio/groundtruth_path`：按首个估计位置和起步段方向对齐，并随 `/vio/path` 最新时间逐步增长的真值轨迹。
- `/camera/stereo_feat_image`：双目特征图。
- `/vio/history_map_points`、`/vio/window_map_points`、`/vio/history_voxels`、`/vio/visit_voxels`：地图点和 voxel 可视化。

## 5. 离线误差评估

完整跑完一个序列后，推荐使用 OpenVINS 风格入口评估：

```bash
ros2 run voxel_svio error_singlerun \
  posyaw \
  /home/scl/work/slam/open_vins/ov_data/euroc_mav/V1_01_easy.txt \
  /tmp/voxel_svio_v1_01/pose.txt
```

该命令使用 `0.02 s` 时间戳最近邻匹配，对估计轨迹执行整段
yaw-only、known-scale 的 `posyaw` Umeyama 对齐，并输出：

- `Absolute Trajectory Error`：姿态误差 `deg` 和位置误差 `m` 的 RMSE、mean、min、max、std。
- `Relative Pose Error`：按 `8, 16, 24, 32, 40 m` 真值距离片段统计的姿态和位置中位误差。
- `Normalized Estimation Error Squared`：若估计轨迹没有 20 列协方差信息，会输出 OpenVINS 风格警告并保持 0 统计。

同一真值和估计文件也可以用 OpenVINS 原生命令并排对比：

```bash
ros2 run ov_eval error_singlerun \
  posyaw \
  /home/scl/work/slam/open_vins/ov_data/euroc_mav/V1_01_easy.txt \
  /tmp/voxel_svio_v1_01/pose.txt
```

旧版位置评估脚本仍然保留，可用于生成对齐轨迹文件：

```bash
ros2 run voxel_svio evaluate_euroc_trajectory.py \
  --estimate /tmp/voxel_svio_v1_01/pose.txt \
  --groundtruth /home/scl/work/slam/open_vins/ov_data/euroc_mav/V1_01_easy.txt \
  --output /tmp/voxel_svio_v1_01/evaluation_V1_01_easy.txt \
  --aligned-output /tmp/voxel_svio_v1_01/aligned_V1_01_easy.txt \
  --max-dt 0.05 \
  --rpe-step 10
```

## 6. 切换序列

只需要替换 `dataset` 和评估时的真值文件名：

```bash
ros2 launch voxel_svio vio_euroc_ros2.launch.py dataset:=V1_02_medium rviz:=true
ros2 launch voxel_svio vio_euroc_ros2.launch.py dataset:=V1_03_difficult rviz:=true
```

每次切换序列建议重新启动 launch，确保估计器状态干净。
