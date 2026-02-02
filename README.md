# lidar_imu_calib

## 简介
一个轻量级的雷达惯性标定工具，专为2D激光雷达与IMU的外参标定设计，核心包含数据采集、时间同步与外参求解三部分。  
数据采集部分实时订阅指定的激光雷达和IMU话题，自动采集一定帧数的有效雷达数据及对应时间段的IMU数据，同时对雷达点云进行极坐标转化直角坐标、无效点过滤等预处理，保证数据的有效性。  
时间同步部分基于球面插值（SLERP）算法，根据雷达帧时间戳对IMU姿态数据进行插值，得到雷达帧时刻对应的IMU旋转矩阵，解决传感器间的时间异步问题。  
外参求解部分通过构建超定方程并结合SVD分解去求解雷达到IMU的旋转矩阵与平移向量，加入正则化项保证求解稳定性，同时计算标定平均残差作为精度评估指标，最终将旋转矩阵、平移向量、平均残差这三个标定结果显示并保存。  
该系统无需依赖复杂的第三方标定工具，可直接在ROS1 Noetic环境下实时运行，适配LubanCat4等ARM架构硬件，能够高效、便捷地完成2D激光雷达与IMU的外参标定。

## 环境
- Ubuntu 20.04 
- ROS1 Noetic 
- Eigen 3.3+
- Minimum C++14
- yaml-cpp 0.6+

## 目录结构
```
lidar_imu_calib
    ├── CMakeLists.txt
    ├── package.xml
    └── src
        └── lidar_imu_calib_node.cpp
```

此目录结构是通过tree命令去生成的，指令如下：
```bash
tree -L 2
```
可以查看2级目录结构，能看到包内的src文件夹中的内容。  
tree命令的安装指令为：
```bash
sudo apt update && sudo apt install -y tree
```

## 详细实施步骤

### step1：创建工作区
在ROS中创建名为lidar_imu_calib_ws的工作区：  
创建工作区目录结构：
```bash
mkdir -p ~/lidar_imu_calib_ws/src
```
初始化工作区：
```bash
cd ~/lidar_imu_calib_ws/src
catkin_init_workspace
```
编译工作区：
```bash
cd ~/lidar_imu_calib_ws
catkin_make
```
添加环境变量：
```bash
source devel/setup.bash
```
  
### step2：创建功能包
```bash
catkin_create_pkg lidar_imu_calib roscpp sensor_msgs pcl_ros pcl_conversions eigen_conversions
```

### step3：雷达惯性标定
对实时采集到的雷达数据和IMU数据进行标定，得到旋转矩阵、平移向量、平均残差三个标定结果，在终端显示并保存至本地。

#### 3.1 编译源码
在工作区根目录编译项目，生成可执行文件：
```bash
cd ~/lidar_imu_calib_ws
catkin_make
```

#### 3.2 加载环境变量 
```bash
source devel/setup.bash
```

#### 3.3 传感器启动
确保所用的2D激光雷达和IMU正常驱动，发布项目所需的话题。  
本项目所使用的2D激光雷达型号为思岚rplidar，参与雷达惯性标定的雷达话题为/scan_correct。  
本项目所使用的IMU为本公司自有九轴产品，型号为DMC620，参与雷达惯性标定的IMU话题为/imu_raw。  
在雷达惯性标定之前两传感器已经做过一次时间软同步，/scan_correct与/imu_raw已经统一到了IMU时间系下。  
可以通过如下命令验证话题是否正常发布：
```bash
rostopic echo --noarr /scan_correct
rostopic echo --noarr /imu_raw
```

#### 3.4 运行标定节点
运行标定节点：
```bash
rosrun lidar_imu_calib lidar_imu_calib_node
```
节点启动后会自动执行以下阶段：  
一、数据采集阶段。本阶段订阅雷达和IMU话题，自动采集一定帧数的有效雷达数据及对应时间段的IMU数据。之后对雷达点云进行预处理，过滤无效点，同时将极坐标转直角坐标。为了便于观察调试，每采集50帧打印一次采集进度，采集完成后自动停止数据接收。  
二、时间同步阶段。本阶段基于球面插值（SLERP）算法，根据雷达帧时间戳对IMU姿态数据插值，精准匹配每个雷达帧时刻对应的IMU旋转矩阵，从而再次进行时间同步，与之前做过的时间软同步形成迭代，提高时间同步的精度。  
三、外参求解阶段。本阶段构建超定方程，加入正则化项保证求解稳定性。之后进行SVD分解求解雷达到IMU的旋转矩阵R_LI和平移向量t_LI。最后计算标定平均残差，作为标定结果评估指标。  
四、结果保存阶段。本阶段将旋转矩阵、平移向量、平均残差显示在终端，并保存到当前目录的calib_result.yaml文件。

#### 3.5 标定结果的使用
标定所得的旋转矩阵R_LI和平移向量t_LI可直接用于激光雷达点云与IMU数据的坐标转换、多传感器融合定位与建图等后续算法的开发等方面。  
标定结果的平均残差可以体现标定精度，帮助使用者判断标定质量。

#### 3.6 核心原理说明
本项目中lidar_imu_calib_node.cpp的核心逻辑分为三部分：  
第一部分是数据采集与预处理。这部分过滤雷达无效点，保证标定数据有效性，同时缓存IMU四元数数据并转换为旋转矩阵，便于后续插值。  
第二部分是时间同步。这部分先找到雷达时间戳在IMU缓存中的前后帧，计算时间比例系数，之后对IMU四元数进行球面插值，得到雷达帧时刻的IMU姿态，实现时间同步。  
第三部分是外参求解与精度评估。这部分构建超定方程Ax=b，通过SVD分解求解雷达到IMU的外参，同时加入正则化项避免矩阵不可逆，保证求解稳定性，此外还计算了世界坐标系下Z轴坐标的平均残差用来评估标定精度，残差越小代表外参越精准。
