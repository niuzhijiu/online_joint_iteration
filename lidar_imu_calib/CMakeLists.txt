cmake_minimum_required(VERSION 3.10)
project(lidar_imu_calib)

## 编译配置
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14 -O3 -march=armv8.2-a -Wall")

## 编译所需的依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
  eigen_conversions
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

## catkin 配置
catkin_package(
  LIBRARIES lidar_imu_calib
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros pcl_conversions eigen_conversions
)

## 头文件目录
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

## 链接目录
link_directories(
  ${PCL_LIBRARY_DIRS}
)

## 可执行文件
add_executable(lidar_imu_calib_node src/lidar_imu_calib_node.cpp)

## 保证消息生成在编译前完成
add_dependencies(lidar_imu_calib_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 链接库
target_link_libraries(lidar_imu_calib_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  yaml-cpp
)

## 安装可执行文件
install(TARGETS lidar_imu_calib_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
