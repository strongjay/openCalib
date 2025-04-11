#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <GeographicLib/LocalCartesian.hpp>
#include <Eigen/Dense>
#include <fstream>
#include <iomanip> 
#include <boost/filesystem.hpp>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <filesystem>
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <sophus/se3.hpp>
#include "geometry_msgs/msg/quaternion.hpp"
#include "geographic_msgs/msg/geo_point.hpp"
#include "geographic_msgs/msg/geo_point.hpp"
#include "geodesy/utm.h"
#include "nav_msgs/msg/odometry.hpp"
using namespace std::chrono_literals;

class OnlineCalib : public rclcpp::Node {
public:
    OnlineCalib() : Node("dataCollector") {
        navsatfix_sub_.subscribe(this, "/fix");
        imu_sub_.subscribe(this, "/imu");
        pointcloud_sub_.subscribe(this, "/rslidar_points");

        sync_ = std::make_shared<Sync>(
            SyncPolicy(100),
            navsatfix_sub_,
            imu_sub_,
            pointcloud_sub_
        );
        sync_->registerCallback(&OnlineCalib::callback, this);

        // 初始化地理坐标转换器
        enu_converter_.Reset(0, 0, 0); // 初始值将在第一帧更新
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/my_odometry", 10);
    }

private:
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::NavSatFix,
        sensor_msgs::msg::Imu,
        sensor_msgs::msg::PointCloud2>;
    using Sync = message_filters::Synchronizer<SyncPolicy>;
    message_filters::Subscriber<sensor_msgs::msg::NavSatFix> navsatfix_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pointcloud_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<Sync> sync_;

    GeographicLib::LocalCartesian enu_converter_;

    void callback(
        const sensor_msgs::msg::NavSatFix::ConstSharedPtr& gps_msg,
        const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg
    ) {
        std::cout<< "get a frame ,timestamp:\n" << std::to_string(rclcpp::Time(gps_msg->header.stamp).seconds()) << std::endl
                                                << std::to_string(rclcpp::Time(imu_msg->header.stamp).seconds()) << std::endl
                                                << std::to_string(rclcpp::Time(cloud_msg->header.stamp).seconds()) << std::endl;
        
        // position
        geographic_msgs::msg::GeoPoint gp;
        gp.latitude  = gps_msg->latitude;
        gp.longitude = gps_msg->longitude;
        gp.altitude  = gps_msg->altitude;
        geodesy::UTMPoint pt(gp);
        Eigen::Vector3d position = Eigen::Vector3d( pt.easting, pt.northing, pt.altitude);

        // orientation
        Eigen::Quaterniond quaterniond_imu(
            imu_msg->orientation.w,
            imu_msg->orientation.x,
            imu_msg->orientation.y,
            imu_msg->orientation.z
        );
        quaterniond_imu.normalize(); 
        Eigen::Matrix3d R_imu = quaterniond_imu.toRotationMatrix();
        
        // save pose
        std::string path = "./debug/pcd/";
        std::string path_pose = path + "pose.txt";
        std::string stamp_str =
            std::to_string(rclcpp::Time(cloud_msg->header.stamp).seconds());
        if (!boost::filesystem::exists(path)) {
            boost::filesystem::create_directories(path);
        }
        std::ofstream fout_pose(path_pose, std::ios::app);
        fout_pose << stamp_str << " " 
                << std::fixed << std::setprecision(8) // 保留 整数部分 和 小数点后8位
                << R_imu(0, 0) << " " << R_imu(0, 1) << " " << R_imu(0, 2) << " " << position.x() << " "
                << R_imu(1, 0) << " " << R_imu(1, 1) << " " << R_imu(1, 2) << " " << position.y() << " "
                << R_imu(2, 0) << " " << R_imu(2, 1) << " " << R_imu(2, 2) << " " << position.z() << " "
                //<< " 0.0 0.0 0.0 1.0" 
                << std::endl;
        
        // save pcd
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*cloud_msg, *cloud);
        std::string file_name = stamp_str + ".pcd";
        std::string file_path = path + "top_center_lidar/";
        if (!boost::filesystem::exists(file_path)) {
            boost::filesystem::create_directories(file_path);
        }
        pcl::io::savePCDFileBinary(file_path + file_name, *cloud);

        // auto gps_odom = nav_msgs::msg::Odometry();
        // gps_odom.header.stamp = this->now();
        // gps_odom.header.frame_id = "rslidar";
        // gps_odom.child_frame_id = "gps";
        // // 设置位置
        // gps_odom.pose.pose.position.x = T_lidar.translation().x();
        // gps_odom.pose.pose.position.y = T_lidar.translation().y();
        // gps_odom.pose.pose.position.z = T_lidar.translation().z();
        // gps_odom.pose.pose.orientation.x = q_rel.x();
        // gps_odom.pose.pose.orientation.y = q_rel.y();
        // gps_odom.pose.pose.orientation.z = q_rel.z();
        // gps_odom.pose.pose.orientation.w = q_rel.w();
    }

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OnlineCalib>());
    rclcpp::shutdown();
    return 0;
}
