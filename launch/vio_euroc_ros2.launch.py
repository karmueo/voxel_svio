"""启动 Voxel-SVIO ROS2 EuRoC Vicon Room 运行、真值叠加和 RViz2 可视化。"""

import os
import shlex

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    """根据 launch 参数创建节点和可选 rosbag2 播放进程。"""
    dataset = LaunchConfiguration("dataset").perform(context)
    bag_root = LaunchConfiguration("bag_root").perform(context)
    groundtruth_root = LaunchConfiguration("groundtruth_root").perform(context)
    output_path = LaunchConfiguration("output_path").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"

    package_share = get_package_share_directory("voxel_svio")
    config_path = os.path.join(package_share, "config", "euroc_ros2.yaml")
    rviz_path = os.path.join(package_share, "rviz_cfg", "euroc_ros2.rviz")
    bag_path = os.path.join(bag_root, dataset)
    groundtruth_path = os.path.join(groundtruth_root, dataset + ".txt")
    resolved_output_path = output_path or os.path.join(os.getcwd(), "output")
    os.makedirs(resolved_output_path, exist_ok=True)

    actions = []
    if not os.path.isdir(bag_path):
        actions.append(LogInfo(msg=f"ERROR: bag path does not exist: {bag_path}"))
        return actions
    if not os.path.isfile(groundtruth_path):
        actions.append(LogInfo(msg=f"ERROR: groundtruth file does not exist: {groundtruth_path}"))
        return actions

    actions.append(
        Node(
            package="voxel_svio",
            executable="vio_node",
            name="vio_node",
            output="screen",
            parameters=[
                config_path,
                {
                    "output_path": resolved_output_path,
                    "use_sim_time": use_sim_time,
                },
            ],
        )
    )

    actions.append(
        Node(
            package="voxel_svio",
            executable="publish_groundtruth_path.py",
            name="groundtruth_path_publisher",
            prefix="/usr/bin/python3",
            output="screen",
            parameters=[
                {
                    "groundtruth_path": groundtruth_path,
                    "estimate_path_topic": "/vio/path",
                    "output_topic": "/vio/groundtruth_path",
                    "frame_id": "camera_init",
                    "use_sim_time": use_sim_time,
                }
            ],
        )
    )

    use_sim_time_text = "true" if use_sim_time else "false"
    rviz_command = (
        f"rviz2 -d {shlex.quote(rviz_path)} --ros-args -p use_sim_time:={use_sim_time_text} & "
        "rviz_pid=$!; "
        "if command -v wmctrl >/dev/null 2>&1; then "
        "for i in $(seq 1 30); do "
        "sleep 0.2; "
        "wmctrl -r RViz -b add,maximized_vert,maximized_horz && break; "
        "wmctrl -r rviz2 -b add,maximized_vert,maximized_horz && break; "
        "done; "
        "fi; "
        "wait $rviz_pid"
    )
    actions.append(
        ExecuteProcess(
            cmd=["bash", "-lc", rviz_command],
            output="screen",
            condition=IfCondition(LaunchConfiguration("rviz")),
        )
    )

    actions.append(
        ExecuteProcess(
            cmd=[
                "ros2",
                "bag",
                "play",
                bag_path,
                "--clock",
                "--topics",
                "/cam0/image_raw",
                "/cam1/image_raw",
                "/imu0",
            ],
            output="screen",
            condition=IfCondition(LaunchConfiguration("play_bag")),
        )
    )

    return actions


def generate_launch_description():
    """生成 Vicon Room ROS2 运行 launch 描述。"""
    return LaunchDescription(
        [
            DeclareLaunchArgument("dataset", default_value="V1_01_easy"),
            DeclareLaunchArgument("bag_root", default_value="/mnt/data/slam/vicon_room1_ros2"),
            DeclareLaunchArgument(
                "groundtruth_root",
                default_value="/home/scl/work/slam/open_vins/ov_data/euroc_mav",
            ),
            DeclareLaunchArgument("output_path", default_value=""),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("play_bag", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            OpaqueFunction(function=launch_setup),
        ]
    )
