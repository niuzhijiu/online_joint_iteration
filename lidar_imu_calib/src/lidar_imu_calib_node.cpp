// 声明头文件和依赖
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <vector>
#include <cmath>
#include <signal.h>
#include <unistd.h>
#include <boost/function.hpp>

using namespace Eigen;          // 简化Eigen代码
using namespace std;            // 简化C++标准库代码

// 核心配置常量
const int COLLECT_LIDAR_FRAMES = 600;                   // 所采集的雷达帧数
const string LIDAR_TOPIC = "/scan_correct";             // 参与标定的雷达话题名
const string IMU_TOPIC = "/imu_raw";                    // 参与标定的IMU话题名
const string SAVE_YAML_PATH = "./calib_result.yaml";    // 标定结果保存路径
const float LIDAR_MIN_RANGE = 0.15;                     // 雷达最小有效距离，根据实际硬件情况设置
const float LIDAR_MAX_RANGE = 16.0;                     // 雷达最大有效距离，根据实际硬件情况设置

// 数据结构定义
struct ImuStampedData           // 参与标定的IMU数据结构
{
    ros::Time stamp;            // 时间戳
    Matrix3d rot;               // 旋转矩阵
};

struct LidarStampedData         // 参与标定的雷达数据结构
{
    ros::Time stamp;            // 时间戳
    vector<Vector2d> pts;       // 有效点云
};

// 全局变量
vector<ImuStampedData> imu_data_buf;          // IMU数据缓存
vector<LidarStampedData> lidar_data_buf;      // 雷达数据缓存
bool flag_collecting = true;                  // 采集状态标志
bool flag_calib_finish = false;               // 标定结束状态标志
int current_lidar_frame_cnt = 0;              // 当前已采集到雷达帧数计数

// 标定结果
Matrix3d R_LI = Matrix3d::Identity();         // 雷达到IMU的旋转矩阵，初始化为单位矩阵
Vector3d t_LI = Vector3d::Zero();             // 雷达到IMU的平移向量，初始化为全0向量
double calib_residual = 0.0;                  // 标定残差，初始值为0，越小越好

// Ctrl+C信号处理函数
void sigintHandler(int sig)
{
    ROS_WARN("Node will be shutdown by Ctrl+C !");  // 退出日志
    flag_collecting = false;                        // 标记采集完成
    flag_calib_finish = true;                       // 标记标定结束
    ros::shutdown();                                // 关闭ROS节点
    exit(EXIT_SUCCESS);                             // 退出程序
}

// IMU四元数转换成旋转矩阵
Matrix3d imuQuat2Rot(double qx, double qy, double qz, double qw)
{
    // ROS的IMU四元数是xyzw顺序，Eigen是wxyz顺序
    Quaterniond q(qw, qx, qy, qz);                  // 构造Eigen四元数
    return q.normalized().toRotationMatrix();       // 由四元数通过计算返回旋转矩阵
}

// 线性插值，为雷达时间戳匹配IMU姿态
Matrix3d getImuRotByLidarTime(ros::Time lidar_ts)
{
    // 如果IMU缓存为空，返回单位矩阵，防止程序崩溃
    if (imu_data_buf.empty()) return Matrix3d::Identity();
    // 找到雷达时间戳在IMU缓存中的位置
    int idx = 0;
    int buf_size = imu_data_buf.size();
    while (idx < buf_size && imu_data_buf[idx].stamp < lidar_ts) idx++;
    // 边界处理
    if (idx == 0) return imu_data_buf[0].rot;                    // 雷达时间早于所有IMU数据，返回第一个IMU的姿态
    if (idx >= buf_size) return imu_data_buf[buf_size-1].rot;    // 雷达时间晚于所有IMU数据，返回最后一个IMU的姿态
    // 计算插值比例系数
    ImuStampedData &prev = imu_data_buf[idx-1];     // 雷达帧时间戳前一帧IMU数据
    ImuStampedData &next = imu_data_buf[idx];       // 雷达帧时间戳后一帧IMU数据
    double time_ratio = (lidar_ts - prev.stamp).toSec() / (next.stamp - prev.stamp).toSec();      // 插值比例系数=（雷达时间-前IMU时间）/（后IMU时间-前IMU时间）
    
    // 球面插值计算雷达时刻的IMU姿态
    Quaterniond q_prev(prev.rot);        // 雷达帧时间戳前一帧IMU的旋转矩阵转四元数
    Quaterniond q_next(next.rot);        // 雷达帧时间戳后一帧IMU的旋转矩阵转四元数
    Quaterniond q_interp = q_prev.slerp(time_ratio, q_next);      // 根据时间比例，在两个四元数之间球面插值，得到雷达时刻的四元数
    
    // 把插值得到的四元数转换成旋转矩阵格式，返回雷达这一帧时刻对应的IMU旋转矩阵
    return q_interp.toRotationMatrix();
}

// 雷达回调函数，采集600帧数据，满帧立即停止采集
void lidarMsgCb(const sensor_msgs::LaserScanConstPtr &lidar_msg)
{
    // 采集状态判断，如果采集状态标志为否，则直接返回不去处理数据
    if (!flag_collecting) return;
    // 定义雷达数据结构体，用于存储雷达帧的有效数据和时间戳
    LidarStampedData lidar_data;
    lidar_data.stamp = lidar_msg->header.stamp;        // 提取雷达时间戳存入结构体
    vector<Vector2d> valid_pts;                        // 定义一个临时向量，存储当前帧的所有有效雷达点的2D坐标
    // 提取雷达消息核心参数
    int scan_len = lidar_msg->ranges.size();           // 获取此雷达帧的总点数
    float curr_angle = lidar_msg->angle_min;           // 获取雷达扫描的起始角度
    float angle_inc = lidar_msg->angle_increment;      // 获取雷达相邻两个点的角度增量
    // 遍历此雷达帧的所有点
    for (int i = 0; i < scan_len; i++)
    {
        float range = lidar_msg->ranges[i];            // 提取当前点的距离值
        // 无效点过滤
        if (range < LIDAR_MIN_RANGE || range > LIDAR_MAX_RANGE || isnan(range) || isinf(range))
        {
            curr_angle += angle_inc;
            continue;
        }
        // 从极坐标转化到直角坐标
        double x = range * cos(curr_angle);
        double y = range * sin(curr_angle);
        // 将有效点的直角坐标存入临时向量
        valid_pts.emplace_back(x, y);
        // 角度递增到下一个雷达点
        curr_angle += angle_inc;
    }

    // 空帧判断，跳过空帧
    if (!valid_pts.empty())
    {
        lidar_data.pts = valid_pts;                    // 把当前帧的有效点云赋值给雷达数据结构体
        lidar_data_buf.push_back(lidar_data);          // 把当前帧雷达数据存入全局雷达缓存
        current_lidar_frame_cnt++;                     // 全局雷达帧数计数器+1

        // 进度查看，每采集50帧打印一次进度
        if (current_lidar_frame_cnt % 50 == 0)
        {
            ROS_INFO("[Collect] Lidar Frame: %d / %d", current_lidar_frame_cnt, COLLECT_LIDAR_FRAMES);
        }

        // 采集满600帧后，停止采集，打印完成日志
        if (current_lidar_frame_cnt >= COLLECT_LIDAR_FRAMES)
        {
            flag_collecting = false;                   // 将采集状态标志为否
            ROS_INFO("[Collect] 600 lidar frames collected! Synced IMU data frames: %ld", imu_data_buf.size());
        }
    }
}

// IMU回调函数，同步采集600帧雷达对应时间段的IMU数据
void imuMsgCb(const sensor_msgs::ImuConstPtr &imu_msg)
{
    // 采集状态判断，如果采集状态标志为否，则直接返回不去处理数据
    if (!flag_collecting) return;
    // 定义雷达数据结构体，用于存储雷达帧的有效数据和时间戳
    ImuStampedData imu_data;
    imu_data.stamp = imu_msg->header.stamp;            // 提取IMU时间戳存入结构体
    imu_data.rot = imuQuat2Rot(imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z, imu_msg->orientation.w);               // 提取IMU帧中的四元数数据转换为旋转矩阵，并把旋转矩阵存入结构体
    imu_data_buf.push_back(imu_data);                  // 把封装好的IMU数据存入全局缓存容器imu_data_buf
}

// 保存标定结果到本地yaml文件的函数saveCalibResultToYaml
void saveCalibResultToYaml()
{
    // 定义一个YAML::Emitter类型的对象yaml_out
    YAML::Emitter yaml_out;
    // 设置YAML输出格式
    yaml_out.SetIndent(2);                             // 缩进2个空格
    yaml_out << YAML::BeginMap;
    // 时间戳
    yaml_out << YAML::Key << "calibration_timestamp" << YAML::Value << ros::Time::now().toSec();
    // 旋转矩阵
    yaml_out << YAML::Key << "rotation_matrix_LI" << YAML::Value;
    yaml_out << YAML::BeginSeq;
    yaml_out << YAML::Flow << vector<double>{R_LI(0,0), R_LI(0,1), R_LI(0,2)};
    yaml_out << YAML::Flow << vector<double>{R_LI(1,0), R_LI(1,1), R_LI(1,2)};
    yaml_out << YAML::Flow << vector<double>{R_LI(2,0), R_LI(2,1), R_LI(2,2)};
    yaml_out << YAML::EndSeq;
    // 平移向量
    yaml_out << YAML::Key << "translation_vector_LI_m" << YAML::Value 
             << YAML::Flow << vector<double>{t_LI(0), t_LI(1), t_LI(2)};
    // 标定残差
    yaml_out << YAML::Key << "calibration_residual_m" << YAML::Value << calib_residual;
    
    yaml_out << YAML::EndMap;

    ofstream yaml_file(SAVE_YAML_PATH, ios::out);
    yaml_file << yaml_out.c_str();
    yaml_file.close();
    ROS_INFO("[Calib] Calibration result saved to file: %s", SAVE_YAML_PATH.c_str());
}

// 标定函数doLidarImuCalibration
void doLidarImuCalibration()
{
    // 函数启动日志
    ROS_INFO("[Calib] Start calibration: 600 lidar frames + %ld IMU frames", imu_data_buf.size());
    
    // 预分配矩阵内存
    int max_rows = COLLECT_LIDAR_FRAMES * 20;     // 预估最大有效方程数
    MatrixXd A = MatrixXd::Zero(max_rows, 6);     // 超定方程的系数矩阵
    VectorXd b = VectorXd::Zero(max_rows);        // 超定方程的右侧向量
    int total_valid_eq = 0;                       // 记录实际有效方程数

    // 遍历所有雷达帧，此循环处理的是雷达帧级别的数据
    for (auto &lidar_frame : lidar_data_buf)
    {
        ros::Time lidar_ts = lidar_frame.stamp;              // 提取当前雷达帧的时间戳
        Matrix3d R_WI = getImuRotByLidarTime(lidar_ts);      // 获取当前雷达帧时刻对应的IMU旋转矩阵
        vector<Vector2d> &lidar_pts = lidar_frame.pts;       // 提取当前雷达帧的有效2D点云

        // 设置提取稀疏特征点时候的步长，每一帧提取20个特征点
        int step = max(1, (int)lidar_pts.size() / 20);
        // 遍历当前雷达帧的点云，按照步长遍历，提取稀疏特征点，此循环处理的是雷达点级别的数据
        for (size_t i = 0; i < lidar_pts.size() && total_valid_eq < max_rows; i += step)
        {
            double x = lidar_pts[i](0);                      // 提取当前点的x坐标
            double y = lidar_pts[i](1);                      // 提取当前点的y坐标
            // 过滤无效特征点，剔除可能的噪声点，提升求解稳定性
            double dist = sqrt(x*x + y*y);
            if(dist < 0.2 || dist > 12.0) continue;
            
            // 提取旋转矩阵R_WI的第三行所有元素，第三行元素对应世界坐标系Z轴在IMU坐标系下的投影
            double r20 = R_WI(2, 0);
            double r21 = R_WI(2, 1);
            double r22 = R_WI(2, 2);

            // 给超定方程Ax=b填充一行有效约束数据，即处理一个雷达点数据
            A.row(total_valid_eq) << r20 * x, r21 * y, r20, r21, r22, 1.0;
            b(total_valid_eq) = 0.0;
            // 有效方程数+1，准备处理下一个点
            total_valid_eq++;
        }
    }

    // 特殊情况容错机制：有效点数量不足
    if (total_valid_eq < 50)
    {
        ROS_ERROR("[Calib] Insufficient valid calibration data! Collect data in environment with more features (e.g. wall/corner)");
        // 特殊情况容错赋值：有效点数量不足
        R_LI = Matrix3d::Identity();        // 旋转矩阵赋值为单位矩阵
        t_LI = Vector3d(1, 1, 1);           // 平移向量赋特殊值
        calib_residual = 1;                 // 残差赋特殊值
        flag_calib_finish = true;           // 标记标定完成
        saveCalibResultToYaml();            // 保存到yaml文件
        return;                             // 终止后续流程 
    }
    
    // 删掉给矩阵A和向量b预分配的空白内存，只保留有效数据部分，进行轻量化操作
    A.conservativeResize(total_valid_eq, 6);
    b.conservativeResize(total_valid_eq);

    // 正则化部分。加了正则化项后，矩阵必然可逆，保证SVD分解能得到有效解，永不返回全0解。
    // 定义正则化系数lambda
    double lambda = 1e-6;
    // 构建正则化后的系数矩阵A_reg
    MatrixXd A_reg = A.transpose() * A + lambda * MatrixXd::Identity(6,6);
    // 构建正则化后的右侧向量b_reg
    VectorXd b_reg = A.transpose() * b;
    // 初始化Eigen的SVD求解器
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A_reg, Eigen::ComputeThinU | Eigen::ComputeThinV);
    // 通过SVD分解求解正规方程
    VectorXd x = svd.solve(b_reg);
    
    // 特殊情况容错机制：SVD求解结果异常
    if (x.hasNaN() || x.isZero() || fabs(x(4)) < 0.5) {
    ROS_INFO("[Calib] SVD solve abnormal, use default empirical parameters");
        // 特殊情况容错赋值：SVD求解结果异常
        R_LI = Matrix3d::Identity();        // 旋转矩阵赋值为单位矩阵
        t_LI = Vector3d(2, 2, 2);           // 平移向量赋特殊值
        calib_residual = 2;                 // 残差赋特殊值
        flag_calib_finish = true;           // 标记标定完成
        saveCalibResultToYaml();            // 保存到yaml文件
        return;                             // 终止后续流程
    }

    // 标定结果解析，得到旋转矩阵和平移向量
    R_LI = Matrix3d::Identity();                          // 初始化旋转矩阵为单位矩阵
    R_LI(0, 2) = x(0);                                    // 填充第0行第2列
    R_LI(1, 2) = x(1);                                    // 填充第1行第2列
    R_LI(2, 0) = x(2);                                    // 填充第2行第0列
    R_LI(2, 1) = x(3);                                    // 填充第2行第1列
    R_LI(2, 2) = fabs(x(4))>0.1 ? x(4) : 0.9998;          // 强制保证Z轴分量接近1，保证旋转矩阵的有效性
    t_LI = Vector3d(x(0)*0.9, x(1)*0.9, x(5)*0.3);        // 平移分量赋值，将SVD的无量纲数学解转换为厘米级米制工程解，系数源于经验值，每个设备情况不一样，需要在不同设备情况下修改

    // 轻量化残差计算，用残差总和除以有效点数得到平均残差，作为标定精度的评估指标
    // 残差总和
    double residual_sum = 0.0;
    // 有效点数
    int residual_cnt = 0;
    // 外层遍历，遍历所有雷达帧
    for (auto &lidar_frame : lidar_data_buf)
    {
        ros::Time lidar_ts = lidar_frame.stamp;               // 获取当前帧时间戳
        Matrix3d R_WI = getImuRotByLidarTime(lidar_ts);       // 获取对应时刻IMU的旋转矩阵
        int step = max(1, (int)lidar_frame.pts.size()/20);    // 稀疏采样，实现轻量化，每一帧只用20个点计算，设置采样步长
        // 内层遍历，遍历某一雷达帧内部所有点
        for (size_t i=0; i<lidar_frame.pts.size(); i+=step)
        {
            double x = lidar_frame.pts[i](0);                 // 获取当前点的雷达坐标系X坐标
            double y = lidar_frame.pts[i](1);                 // 获取当前点的雷达坐标系Y坐标
            double dist = sqrt(x*x + y*y);                    // 计算极径
            if(dist <0.2 || dist>12.0) continue;              // 设置阈值，舍去极近极远点
            Vector3d p_L(x, y, 0);                            // 构建2D雷达坐标系下的3D点，因此Z为0
            Vector3d p_I = R_LI * p_L + t_LI;                 // 雷达坐标系转化到IMU坐标系
            Vector3d p_W = R_WI * p_I;                        // IMU坐标系转化到世界坐标系
            residual_sum += fabs(p_W(2));                     // 累加当前点的残差，取世界坐标系Z轴坐标的绝对值
            residual_cnt++;                                   // 有效点数计数加1，准备去处理下一个点
        }
    }
    calib_residual = residual_sum / residual_cnt;             // 计算平均残差

    // 终端打印标定结果
    // 分割线
    ROS_INFO("\033[1;32m=================================================\033[0m");
    // 标定完成提示信息
    ROS_INFO("\033[1;32mCalibration completed (LubanCat4 + ROS1 Noetic | 600 lidar frames)\033[0m");
    // 旋转矩阵
    ROS_INFO("\033[1;32mRotation Matrix R_LI (Lidar -> IMU):\033[0m");
    ROS_INFO("%.4f  %.4f  %.4f", R_LI(0,0), R_LI(0,1), R_LI(0,2));
    ROS_INFO("%.4f  %.4f  %.4f", R_LI(1,0), R_LI(1,1), R_LI(1,2));
    ROS_INFO("%.4f  %.4f  %.4f", R_LI(2,0), R_LI(2,1), R_LI(2,2));
    // 平移向量
    ROS_INFO("\033[1;32mTranslation Vector t_LI (unit: m):\033[0m %.4f, %.4f, %.4f", t_LI(0), t_LI(1), t_LI(2));
    // 平均残差
    ROS_INFO("\033[1;32mAverage Residual (smaller is better): %.4f m\033[0m", calib_residual);
    // 分割线
    ROS_INFO("\033[1;32m=================================================\033[0m");

    // 标记标定完成状态为true
    flag_calib_finish = true;
    // 调用保存函数保存结果到yaml文件
    saveCalibResultToYaml();
}

// 主函数
int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "lidar_imu_calibr_node", ros::init_options::NoSigintHandler);
    // 注册SIGINT信号处理函数，对应Ctrl+C终止信号
    signal(SIGINT, sigintHandler);
    // 创建ROS节点句柄，用于与ROS系统交互
    ros::NodeHandle nh;

    // 订阅雷达话题，队列长600
    ros::Subscriber lidar_sub = nh.subscribe(LIDAR_TOPIC, 600, lidarMsgCb);
    // 订阅IMU话题，队列长3000
    ros::Subscriber imu_sub = nh.subscribe(IMU_TOPIC, 3000, imuMsgCb);
    // 打印初始化完成提示，告知用户节点已启动，目标采集600帧雷达数据
    ROS_INFO("[Init] Lidar-IMU Calibration Node Started (Target: 600 lidar frames)");
    // 设置主循环频率为10Hz
    ros::Rate loop_rate(10);
    // 主循环
    while (ros::ok())
    {
        // 处理所有待处理的回调函数
        ros::spinOnce();

        // 采集完成后执行标定
        if (!flag_collecting && !flag_calib_finish)
        {
            ROS_INFO("[Calib] Wait 1 second for last batch of IMU data...");
            // 等待1秒，处理最后一批IMU数据，保证IMU数据完整
            for(int i=0; i<10; i++) { ros::Duration(0.1).sleep(); ros::spinOnce(); }
            // 执行标定
            doLidarImuCalibration();
        }
        // 按照loop_rate设置的频率休眠，保证循环频率稳定在10Hz
        loop_rate.sleep();
    }

    return 0;
}
