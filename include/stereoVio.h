/**
 * @file stereoVio.h
 * @brief 声明 ROS2 版 Voxel-SVIO 双目视觉惯性里程计节点。
 */

#pragma once

// c++ include
#include <chrono>
#include <iostream>
#include <math.h>
#include <thread>
#include <fstream>
#include <vector>
#include <queue>
#include <unordered_set>

// lib include
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp> 
#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// function include
#include "utility.h"
#include "parameters.h"
#include "sensorData.h"
#include "state.h"
#include "stateHelper.h"
#include "msckf.h"
#include "feature.h"
#include "featureTracker.h"
#include "mapPoint.h"
#include "mapManagement.h"
#include "initializer.h"
#include "frame.h"

struct Measurements
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::vector<imuData> imu_measurements;
    cameraData image_measurements;
};

/**
 * @brief ROS2 双目视觉惯性里程计节点。
 */
class voxelStereoVio : public rclcpp::Node
{
private:

    /**
     * @brief 处理单条 IMU 数据并送入传播器和初始化器。
     * @param imu_data IMU 采样数据。
     */
    void processImu(imuData &imu_data);

    /**
     * @brief 处理一帧双目图像并触发初始化或滤波更新。
     * @param image_measurements_const 双目图像测量。
     */
    void processImage(cameraData &image_measurements_const);

    /**
     * @brief 使用当前图像测量执行 MSCKF/SLAM 更新。
     * @param image_measurements 双目图像测量。
     */
    void featureUpdate(cameraData &image_measurements);

    /**
     * @brief 三角化当前活跃特征轨迹。
     * @param image_measurements 双目图像测量。
     */
    void triangulateActiveTracks(cameraData &image_measurements);

    /**
     * @brief 查询最近访问的 voxel 用于可视化。
     * @param timestamp 当前时间戳。
     * @param voxels_visit 输出的最近访问 voxel 点云。
     */
    void getRecentVoxel(double timestamp, pcl::PointCloud<pcl::PointXYZI>::Ptr voxels_visit);

    /**
     * @brief 尝试用缓存图像和 IMU 初始化 VIO 状态。
     * @param image_measurements 当前图像测量。
     * @return 初始化成功返回 true。
     */
    bool tryToInitialize(cameraData &image_measurements);

    /**
     * @brief 声明并读取一个 ROS2 参数。
     * @tparam T 参数类型。
     * @param name 参数名称。
     * @param default_value 默认值。
     * @return 参数值。
     */
    template <typename T>
    T declareAndGetParameter(const std::string &name, const T &default_value)
    {
        this->declare_parameter<T>(name, default_value);
        return this->get_parameter(name).get_value<T>();
    }

    /**
     * @brief 声明并读取整数数组参数。
     * @param name 参数名称。
     * @param default_value 默认整数数组。
     * @return 转换为 int 的参数数组。
     */
    std::vector<int> declareAndGetIntVectorParameter(const std::string &name, const std::vector<int> &default_value)
    {
        std::vector<int64_t> default_value_i64(default_value.begin(), default_value.end());
        this->declare_parameter<std::vector<int64_t>>(name, default_value_i64);
        std::vector<int64_t> value_i64 = this->get_parameter(name).get_value<std::vector<int64_t>>();
        return std::vector<int>(value_i64.begin(), value_i64.end());
    }

    /**
     * @brief 创建 ROS2 订阅、发布和定时器。
     */
    void setupRosInterfaces();

    /**
     * @brief 清空本次运行的轨迹输出文件。
     */
    void resetPoseOutputFile();

    std::queue<cameraData> camera_buffer;

    std::queue<imuData> imu_buffer;

    std::map<int, double> camera_last_timestamp;

    odometryOptions odometry_options;

    std::shared_ptr<trackKLT> featureTracker;

    std::shared_ptr<inertialInitializer> initializer_ptr;

    double startup_time;

    double current_time;

    double time_newest_imu;

    std::vector<std::pair<double, std::pair<Eigen::Vector3d, Eigen::Vector3d>>> imu_meas;
    std::vector<imuState> imu_states;

    std::shared_ptr<state> state_ptr;

    imuData last_imu_data;

    std::shared_ptr<propagator> propagator_ptr;

    std::shared_ptr<updaterMsckf> updaterMsckf_ptr;

    std::shared_ptr<updaterSlam> updaterSlam_ptr;

    std::shared_ptr<gammaPixel> gammaPixel_ptr;

    std::shared_ptr<frame> newest_fh;

    std::vector<double> camera_queue_init;

    std::vector<std::shared_ptr<frame>> frame_queue_init;

    voxelHashMap voxel_map;

    int frame_count;

    MarginalizeStatus marginalize_status;

    double last_time_image;

    std::vector<Eigen::Vector3d> good_features_msckf;

    double active_tracks_time = -1;
    std::unordered_map<size_t, Eigen::Vector3d> active_tracks_pos_world; // active_tracks_posinG
    std::unordered_map<size_t, Eigen::Vector3d> active_tracks_pos_world_new;
    std::unordered_map<size_t, Eigen::Vector3d> active_tracks_uvd;
    cv::Mat active_image;
    std::map<size_t, Eigen::Matrix3d> active_feat_linsys_A;
    std::map<size_t, Eigen::Vector3d> active_feat_linsys_b;
    std::map<size_t, int> active_feat_linsys_count;

    std::vector<Eigen::Matrix<short, 3, 1>> recent_voxels;

    std::ofstream of_statistics;
    boost::posix_time::ptime rT1, rT2, rT3, rT4, rT5, rT6, rT7;

    double timelastupdate = -1;
    double distance = 0;

    // time test
    double sum_time_1 = 0.0;
    double sum_time_2 = 0.0;
    double sum_time_3 = 0.0;
    double sum_time_4 = 0.0;
    double sum_time_5 = 0.0;
    double sum_time_6 = 0.0;
    double sum_time_7 = 0.0;

    double sum_time_sum = 0.0;

    int n_time_1 = 0;
    int n_time_2 = 0;
    int n_time_3 = 0;
    int n_time_4 = 0;
    int n_time_5 = 0;
    int n_time_6 = 0;
    int n_time_7 = 0;

    int n_time_sum = 0;
    // time test

public:

    /**
     * @brief 构造 Voxel-SVIO ROS2 节点。
     * @param options ROS2 节点选项。
     */
    explicit voxelStereoVio(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    /**
     * @brief 从 ROS2 参数服务器读取算法参数。
     */
    void readParameters();

    /**
     * @brief 分配算法运行所需对象和缓存。
     */
    void allocateMemory();

    /**
     * @brief 初始化运行时状态变量。
     */
    void initialValue();

    /**
     * @brief ROS2 IMU 回调。
     * @param msg IMU 消息。
     */
    void imuHandler(const sensor_msgs::msg::Imu::ConstSharedPtr msg);

    /**
     * @brief ROS2 双目图像同步回调。
     * @param msg_0 左目图像。
     * @param msg_1 右目图像。
     * @param cam_0 左目相机编号。
     * @param cam_1 右目相机编号。
     */
    void stereoImageHandler(const sensor_msgs::msg::Image::ConstSharedPtr msg_0, const sensor_msgs::msg::Image::ConstSharedPtr msg_1, int cam_0, int cam_1);

    /**
     * @brief 周期性处理缓存图像和 IMU 数据。
     */
    void run();

    std::string image_left_topic;
    std::string image_right_topic;
    std::string imu_topic;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncStereoImage;
    std::vector<std::shared_ptr<message_filters::Synchronizer<SyncStereoImage>>> sync_cam;
    std::vector<std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>>> sync_subs_cam;

    // display
    /**
     * @brief 发布带特征可视化的双目图像。
     * @param stereo_image 双目拼接图像。
     * @param timestamp 图像时间戳。
     */
    void pubFeatImage(cv::Mat &stereo_image, double &timestamp);

    /**
     * @brief 发布当前里程计位姿。
     * @param state_ptr 当前滤波状态。
     * @param timestamp 当前时间戳。
     */
    void pubOdometry(std::shared_ptr<state> state_ptr, double &timestamp);

    /**
     * @brief 将滤波状态写入 PoseStamped。
     * @param body_pose_out 输出位姿消息。
     * @param state_ptr 当前滤波状态。
     */
    void setPoseStamp(geometry_msgs::msg::PoseStamped &body_pose_out, std::shared_ptr<state> state_ptr);

    /**
     * @brief 发布当前累计轨迹。
     * @param state_ptr 当前滤波状态。
     * @param timestamp 当前时间戳。
     */
    void pubPath(std::shared_ptr<state> state_ptr, double &timestamp);

    /**
     * @brief 发布历史地图点云。
     * @param points_history 历史地图点。
     * @param timestamp 当前时间戳。
     */
    void pubHistoryPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_history, double &timestamp);

    /**
     * @brief 发布滑动窗口地图点云。
     * @param state_ptr 当前滤波状态。
     * @param timestamp 当前时间戳。
     */
    void pubWindowPoints(std::shared_ptr<state> state_ptr, double &timestamp);

    /**
     * @brief 发布历史 voxel 点云。
     * @param voxels_history 历史 voxel。
     * @param timestamp 当前时间戳。
     */
    void pubHistoryVoxels(pcl::PointCloud<pcl::PointXYZI>::Ptr voxels_history, double &timestamp);

    /**
     * @brief 发布最近访问 voxel 点云。
     * @param voxels_visit 最近访问 voxel。
     * @param timestamp 当前时间戳。
     */
    void pubVisitVoxels(pcl::PointCloud<pcl::PointXYZI>::Ptr voxels_visit, double &timestamp);

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_feat_image;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_points_history;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_points_window;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_voxels_history;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_voxels_visit;

    geometry_msgs::msg::PoseStamped msg_body_pose;
    nav_msgs::msg::Odometry odom;
    nav_msgs::msg::Path path;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_history;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_window;
    pcl::PointCloud<pcl::PointXYZI>::Ptr voxels_history;
    pcl::PointCloud<pcl::PointXYZI>::Ptr voxels_visit;
    rclcpp::TimerBase::SharedPtr timer;
    // display
};
