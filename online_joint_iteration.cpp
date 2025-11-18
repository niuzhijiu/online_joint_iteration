#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <vector>
#include <mutex>
#include <cmath>
#include <algorithm>
#include <thread>

// IMU数据结构
struct ImuSample {
    double t;
    double gx, gy, gz;
};

// 图像数据结构
struct ImageSample {
    double t;
    cv::Mat img;
};

// 视觉角速度数据结构
struct VisSample {
    double t;
    double wx, wy, wz;
};

// 全局缓存
std::vector<ImuSample> imu_buffer;
std::vector<ImageSample> image_buffer;
std::mutex buf_mutex;


// 偏移历史
std::vector<double> bias_history;

// ROS发布者
ros::Publisher pub_correct;

// ------------------ 工具函数 ------------------
// 计算视觉角速度
std::vector<VisSample> computeVisualAngularVelocity(const ImageSample& img1, const ImageSample& img2) {
    std::vector<VisSample> vis;
    // 相机内参
    cv::Mat K = (cv::Mat_<double>(3,3) << 685.0, 0.0, 320.0, 0.0, 685.0, 240.0, 0.0, 0.0, 1.0);
    cv::Ptr<cv::ORB> orb = cv::ORB::create(2000);

    std::vector<cv::KeyPoint> kp1, kp2;
    cv::Mat des1, des2;
    orb->detectAndCompute(img1.img, cv::Mat(), kp1, des1);
    orb->detectAndCompute(img2.img, cv::Mat(), kp2, des2);
    if (des1.empty() || des2.empty()) return vis;

    cv::BFMatcher bf(cv::NORM_HAMMING, true);
    std::vector<cv::DMatch> matches;
    bf.match(des1, des2, matches);
    if (matches.size() < 5) return vis;

    std::vector<cv::Point2f> pts1, pts2;
    for (auto &m : matches) {
        pts1.push_back(kp1[m.queryIdx].pt);
        pts2.push_back(kp2[m.trainIdx].pt);
    }

    cv::Mat mask;
    cv::Mat E = cv::findEssentialMat(pts1, pts2, K, cv::RANSAC, 0.999, 1.0, mask);
    if (E.empty() || cv::countNonZero(mask) < 5) return vis;

    cv::Mat R, tvec;
    cv::recoverPose(E, pts1, pts2, K, R, tvec, mask);
    cv::Mat rvec;
    cv::Rodrigues(R, rvec);

    double dt = img2.t - img1.t;
    if (dt <= 0.0) return vis;

    cv::Vec3d omega(rvec.at<double>(0,0)/dt, rvec.at<double>(1,0)/dt, rvec.at<double>(2,0)/dt);
    VisSample v;
    v.t = 0.5*(img1.t + img2.t);
    v.wx = omega[0]; v.wy = omega[1]; v.wz = omega[2];
    vis.push_back(v);

    return vis;
}

// 筛选IMU时间段
std::vector<ImuSample> filterImuInRange(const std::vector<ImuSample>& imu_all, double t0, double t1) {
    std::vector<ImuSample> out;
    for (auto& s : imu_all)
        if (s.t >= t0 && s.t <= t1)
            out.push_back(s);
    return out;
}

// 插值视觉到IMU时间序列
std::vector<double> interpolateVisToImu(const std::vector<VisSample>& vis,
                                        const std::vector<double>& imu_t,
                                        const std::vector<double>& vis_s,
                                        double bias) {
    std::vector<double> vis_interp;
    vis_interp.reserve(imu_t.size());
    if (vis.empty()) {
        vis_interp.assign(imu_t.size(), 0.0);
        return vis_interp;
    }
    for (double t : imu_t) {
        double shifted_t = t - bias;
        size_t j = 0;
        while (j+1 < vis.size() && vis[j+1].t < shifted_t) ++j;
        if (j+1 >= vis.size()) { vis_interp.push_back(vis_s.back()); continue; }
        double t1 = vis[j].t, t2 = vis[j+1].t;
        double v1 = vis_s[j], v2 = vis_s[j+1].t;  // 注意用vis_s[j+1]的值
        double alpha = (t2 - t1 > 1e-12) ? (shifted_t - t1)/(t2 - t1) : 0.0;
        vis_interp.push_back(v1 + alpha*(v2 - v1));
    }
    return vis_interp;
}

// 归一化互相关
double computeCorrelation(const std::vector<double>& imu_s, const std::vector<double>& vis_s) {
    if (imu_s.size() != vis_s.size() || imu_s.empty()) return 0.0;
    double mean_imu=0.0, mean_vis=0.0;
    for (size_t i=0;i<imu_s.size();i++){ mean_imu+=imu_s[i]; mean_vis+=vis_s[i]; }
    mean_imu/=imu_s.size(); mean_vis/=vis_s.size();
    double num=0.0, denom1=0.0, denom2=0.0;
    for (size_t i=0;i<imu_s.size();i++){
        double a=imu_s[i]-mean_imu;
        double b=vis_s[i]-mean_vis;
        num+=a*b; denom1+=a*a; denom2+=b*b;
    }
    if(denom1<=0.0 || denom2<=0.0) return 0.0;
    return num/std::sqrt(denom1*denom2);
}

// 根据一帧增量段估计偏移
double estimateBiasIncremental(const ImageSample& prev_img, const ImageSample& cur_img,
                               const std::vector<ImuSample>& imu_all, double prev_bias) {
    auto vis = computeVisualAngularVelocity(prev_img, cur_img);
    if(vis.empty()) return prev_bias;

    auto imu_seg = filterImuInRange(imu_all, prev_img.t + prev_bias, cur_img.t + prev_bias);
    if(imu_seg.size()<2) return prev_bias;

    // 提取IMU时间序列和视觉角速度
    std::vector<double> imu_x, vis_x;
    std::vector<double> imu_t;
    for(auto& s: imu_seg){ imu_x.push_back(s.gx); imu_t.push_back(s.t); }
    for(auto& v: vis){ vis_x.push_back(v.wx); }

    // 搜索偏移 [-25ms,25ms]
    double best_bias=prev_bias; double best_rho=-1.0;
    for(double b=-0.025;b<=0.025;b+=0.001){
        auto vis_interp=interpolateVisToImu(vis, imu_t, vis_x, b);
        double rho=computeCorrelation(imu_x, vis_interp);
        if(rho>best_rho){ best_rho=rho; best_bias=b; }
    }
    return best_bias;
}

// ------------------ ROS回调 ------------------
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    ImuSample s;
    s.t = msg->header.stamp.toSec();
    s.gx = msg->angular_velocity.x;
    s.gy = msg->angular_velocity.y;
    s.gz = msg->angular_velocity.z;
    std::lock_guard<std::mutex> lock(buf_mutex);
    imu_buffer.push_back(s);
}

void imageCallback(const sensor_msgs::Image::ConstPtr& msg){
    double t = msg->header.stamp.toSec();
    cv::Mat img;
    try{
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg,"bgr8");
        img = cv_ptr->image.clone();
    } catch(...){ return; }

    std::lock_guard<std::mutex> lock(buf_mutex);

    ImageSample cur_img; cur_img.t=t; cur_img.img=img;
    image_buffer.push_back(cur_img);

    if(image_buffer.size()<2){
        // 第一帧直接发布
        pub_correct.publish(msg);
        return;
    }

    // 增量计算偏移
    auto& prev_img = image_buffer[image_buffer.size()-2];
    double prev_bias = bias_history.empty() ? 0.0 : bias_history.back();
    double new_bias = estimateBiasIncremental(prev_img, cur_img, imu_buffer, prev_bias);
    bias_history.push_back(new_bias);

    // 修正时间戳并发布
    std_msgs::Header hdr;
    hdr.stamp = ros::Time(t + new_bias);
    cv_bridge::CvImage cv_out(hdr, "bgr8", img);
    pub_correct.publish(cv_out.toImageMsg());
}

// ------------------ 主函数 ------------------
int main(int argc, char** argv){
    ros::init(argc, argv, "timestamp_correct_node");
    ros::NodeHandle nh;

    ros::Subscriber sub_imu = nh.subscribe("/imu_raw", 20000, imuCallback);
    ros::Subscriber sub_img = nh.subscribe("/image_timestamp_raw", 2000, imageCallback);
    pub_correct = nh.advertise<sensor_msgs::Image>("/image_correct_raw",2000);

    ROS_INFO("Online joint iterative timestamp correction node started.");
    ros::spin();
    return 0;
}
