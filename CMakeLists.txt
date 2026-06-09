# ROS2 版本构建配置，用于编译 Voxel-SVIO 节点并安装运行资源。
cmake_minimum_required(VERSION 3.8)
project(voxel_svio)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)
find_package(Boost REQUIRED COMPONENTS date_time filesystem system thread)

find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include(FetchContent)

FetchContent_Declare(
  tessil
  SOURCE_DIR ${PROJECT_SOURCE_DIR}/thirdLibrary/tessil-src
)

if(NOT tessil_POPULATED)
  FetchContent_Populate(tessil)

  add_library(robin_map INTERFACE)
  add_library(tsl::robin_map ALIAS robin_map)

  target_include_directories(robin_map INTERFACE
    "$<BUILD_INTERFACE:${tessil_SOURCE_DIR}/include>"
  )

  list(APPEND headers
    "${tessil_SOURCE_DIR}/include/tsl/robin_growth_policy.h"
    "${tessil_SOURCE_DIR}/include/tsl/robin_hash.h"
    "${tessil_SOURCE_DIR}/include/tsl/robin_map.h"
    "${tessil_SOURCE_DIR}/include/tsl/robin_set.h"
  )
  target_sources(robin_map INTERFACE "$<BUILD_INTERFACE:${headers}>")
endif()

add_executable(
  vio_node
  src/camera.cpp
  src/frame.cpp
  src/pixelSelector.cpp
  src/feature.cpp
  src/featureHelper.cpp
  src/featureTracker.cpp
  src/initializer.cpp
  src/initializerHelper.cpp
  src/mapPoint.cpp
  src/mapManagement.cpp
  src/msckf.cpp
  src/parameters.cpp
  src/preIntegration.cpp
  src/quatOps.cpp
  src/sensorData.cpp
  src/state.cpp
  src/stateHelper.cpp
  src/stereoVio.cpp
  src/updaterHelper.cpp
  src/utility.cpp
)

target_include_directories(vio_node PRIVATE
  include
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(vio_node
  cv_bridge
  geometry_msgs
  message_filters
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
)

target_link_libraries(vio_node
  ${Boost_LIBRARIES}
  ${CERES_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  robin_map
)

install(TARGETS vio_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/error_singlerun.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME error_singlerun
)

install(PROGRAMS
  scripts/evaluate_euroc_trajectory.py
  scripts/publish_groundtruth_path.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  rviz_cfg
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  launch/vio_euroc_ros2.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest QUIET)
  if(ament_cmake_pytest_FOUND)
    ament_add_pytest_test(test_error_singlerun test/test_error_singlerun.py)
    ament_add_pytest_test(test_evaluate_euroc_trajectory test/test_evaluate_euroc_trajectory.py)
    ament_add_pytest_test(test_euroc_ros2_config test/test_euroc_ros2_config.py)
    ament_add_pytest_test(test_publish_groundtruth_path test/test_publish_groundtruth_path.py)
    ament_add_pytest_test(test_stereo_vio_output_policy test/test_stereo_vio_output_policy.py)
    ament_add_pytest_test(test_static_initializer_policy test/test_static_initializer_policy.py)
  endif()
endif()

ament_package()
