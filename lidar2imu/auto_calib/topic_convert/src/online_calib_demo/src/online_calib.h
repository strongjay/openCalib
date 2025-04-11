#ifndef ONLINE_CALIB_H
#define ONLINE_CALIB_H

#include <pcl/point_types.h>



struct LidarPointXYZI {
  PCL_ADD_POINT4D;
  float intensity;
//   uint16_t ring;
//   double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    LidarPointXYZI,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
    // (uint16_t, ring, ring)(double, timestamp, timestamp)
    )


// 自定义点云类型
// struct LidarPointXYZIRT {
//   PCL_ADD_POINT4D;  // 自动添加 x,y,z,w 并处理内存对齐
//   float intensity;
//   uint16_t ring;
//   double timestamp;
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // 确保 Eigen 内存对齐
// } EIGEN_ALIGN16;  // 强制 16 字节对齐

// 注册点云类型
// POINT_CLOUD_REGISTER_POINT_STRUCT(
//   LidarPointXYZIRT,
//   (float, x, x)
//   (float, y, y)
//   (float, z, z)
//   (float, intensity, intensity)
//   (uint16_t, ring, ring)
//   (double, timestamp, timestamp)
// )




// 其他头文件
#include <memory>
#include <queue>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/point_cloud.h>
#include <sophus/se3.hpp>
#include <boost/filesystem.hpp>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <filesystem>
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace localization {
namespace onlinecalib {

class OnlineCalib : public rclcpp::Node {
public:
    OnlineCalib();

private:
    void TimeSyncCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr front_left_msg,
                          const nav_msgs::msg::Odometry::ConstSharedPtr gnss_odom_msg);
    void timer_callback();

    // Topics and parameters
    std::string ns_;
    std::string lidar_topic_;
    std::string imu_topic_;
    bool is_calib_init_;

    // Subscribers
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> lid_front_left_sub_;
    message_filters::Subscriber<nav_msgs::msg::Odometry> gnss_odom_sub_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync_policy_;

    // Queues for data synchronization
    std::queue<pcl::PointCloud<LidarPointXYZI>::Ptr> lidar_queue_;
    std::queue<nav_msgs::msg::Odometry::ConstSharedPtr> gnss_odom_queue_;

    // Transformation matrix from initial pose to current pose
    Sophus::SE3d T_lidar_init;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace onlinecalib
} // namespace localization

#endif // ONLINE_CALIB_H