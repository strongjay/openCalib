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
    bool first_frame_ = true;
    Eigen::Vector3d initial_position_;
    Eigen::Matrix3d initial_rotation_;

    bool init_ = false;
    double init_lat_, init_lon_, init_alt_;
    double init_x_, init_y_, init_z_;
    double fix_x_, fix_y_, fix_z_;
    tf2::Quaternion init_orientation_;
    tf2::Quaternion current_orientation_;
    Sophus::SE3d T_lidar_init;

    void callback(
        const sensor_msgs::msg::NavSatFix::ConstSharedPtr& gps_msg,
        const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg
    ) {
        std::cout<< "get a frame ,timestamp:\n" << std::to_string(rclcpp::Time(gps_msg->header.stamp).seconds()) << std::endl
                                                << std::to_string(rclcpp::Time(imu_msg->header.stamp).seconds()) << std::endl
                                                << std::to_string(rclcpp::Time(cloud_msg->header.stamp).seconds()) << std::endl;
        if(!init_){
            init_ = true;
            // orientatoin
            init_orientation_.setX(imu_msg->orientation.x);
            init_orientation_.setY(imu_msg->orientation.y);
            init_orientation_.setZ(imu_msg->orientation.z);
            init_orientation_.setW(imu_msg->orientation.w);
            Eigen::Quaterniond rotation = Eigen::Quaterniond(   init_orientation_.w(),
                                                                init_orientation_.x(),
                                                                init_orientation_.y(),
                                                                init_orientation_.z()
                                                                );
            // position
            geographic_msgs::msg::GeoPoint gp_init;
            gp_init.latitude = gps_msg->latitude;
            gp_init.longitude = gps_msg->longitude;
            gp_init.altitude = gps_msg->altitude;
            geodesy::UTMPoint pt_init(gp_init);
            init_x_ = pt_init.easting;
            init_y_ = pt_init.northing;
            init_z_ = pt_init.altitude;
            Eigen::Vector3d position = Eigen::Vector3d( init_x_, init_y_, init_z_);
            Sophus::SE3d current_pose(rotation, position);
            T_lidar_init = current_pose;
            RCLCPP_INFO(this->get_logger(), "init done!");
        }
        // orientation
        current_orientation_.setX(imu_msg->orientation.x);
        current_orientation_.setY(imu_msg->orientation.y);
        current_orientation_.setZ(imu_msg->orientation.z);
        current_orientation_.setW(imu_msg->orientation.w);
        current_orientation_.normalize();
        tf2::Quaternion z_rot;
        z_rot.setRPY(0, 0, M_PI*90/180);  // 绕Z轴旋转-90度
        tf2::Quaternion q_rel = current_orientation_ ;//* z_rot;
        q_rel.normalize();
        Eigen::Quaterniond rotation = Eigen::Quaterniond(   q_rel.w(),
                                                            q_rel.x(),
                                                            q_rel.y(),
                                                            q_rel.z()
                                                            );
        // position
        geographic_msgs::msg::GeoPoint gp;
        gp.latitude  = gps_msg->latitude;
        gp.longitude = gps_msg->longitude;
        gp.altitude  = gps_msg->altitude;
        geodesy::UTMPoint pt(gp);
        fix_x_ = pt.easting;  
        fix_y_ = pt.northing;
        fix_z_ = pt.altitude;
        Eigen::Vector3d position = Eigen::Vector3d( fix_x_, fix_y_, fix_z_);
        
        // Rt
        Sophus::SE3d current_pose(rotation, position);
        Sophus::SE3d T_lidar = T_lidar_init.inverse() * current_pose;
        Eigen::Matrix3d pose_matrix3d = T_lidar.rotationMatrix();
        // save pose
        std::string path = "./debug/pcd/";
        std::string path_pose = path + "pose.txt";
        std::string stamp_str =
            std::to_string(rclcpp::Time(cloud_msg->header.stamp).seconds());
        if (!boost::filesystem::exists(path)) {
            boost::filesystem::create_directories(path);
        }
        std::ofstream fout_pose(path_pose, std::ios::app);
        fout_pose << stamp_str << " " << pose_matrix3d(0, 0) << " "
              << pose_matrix3d(0, 1) << " " << pose_matrix3d(0, 2) << " "
              << T_lidar.translation().x() << " " << pose_matrix3d(1, 0) << " "
              << pose_matrix3d(1, 1) << " " << pose_matrix3d(1, 2) << " "
              << T_lidar.translation().y() << " " << pose_matrix3d(2, 0) << " "
              << pose_matrix3d(2, 1) << " " << pose_matrix3d(2, 2) << " "
              << T_lidar.translation().z() 
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

    // void setup_initial_frame(
    //     const sensor_msgs::msg::NavSatFix::ConstSharedPtr& gps_msg,
    //     const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg
    // ) {
    //     enu_converter_.Reset(gps_msg->latitude, gps_msg->longitude, gps_msg->altitude);
    //     double east, north, up;
    //     enu_converter_.Forward(gps_msg->latitude, gps_msg->longitude, gps_msg->altitude, east, north, up);
    //     initial_position_ = Eigen::Vector3d(east, north, up);
        
    //     Eigen::Quaterniond q(
    //         imu_msg->orientation.w,
    //         imu_msg->orientation.x,
    //         imu_msg->orientation.y,
    //         imu_msg->orientation.z
    //     );
    //     initial_rotation_ = q.normalized().toRotationMatrix();
    // }

    // std::pair<Eigen::Vector3d, Eigen::Matrix3d> get_current_pose(
    //     const sensor_msgs::msg::NavSatFix::ConstSharedPtr& gps_msg,
    //     const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg
    // ) {
    //     double east, north, up;
    //     enu_converter_.Forward(gps_msg->latitude, gps_msg->longitude, gps_msg->altitude, east, north, up);
    //     Eigen::Vector3d position(east, north, up);

    //     Eigen::Quaterniond q(
    //         imu_msg->orientation.w,
    //         imu_msg->orientation.x,
    //         imu_msg->orientation.y,
    //         imu_msg->orientation.z
    //     );
    //     Eigen::Matrix3d rotation = q.normalized().toRotationMatrix();    
    
    //     return {position, rotation}; 
    // }

    // Eigen::Matrix3d calculate_inverse_rotation(const Eigen::Matrix3d& current_rot) {
    //     Eigen::Matrix3d relative_rot = initial_rotation_.transpose() * current_rot;
    //     return relative_rot.inverse();
    // }

    // void save_transform_matrix(const Eigen::Matrix3d& rotation,
    //                            const Eigen::Vector3d& translation,
    //                            const std_msgs::msg::Header::_stamp_type& stamp) 
    // {
    //     std::string path = "./pcd/";
    //     std::string path_pose = path + "pose.txt";
    //     std::string stamp_str =
    //         std::to_string(rclcpp::Time(stamp).seconds());
    //     if (!boost::filesystem::exists(path)) {
    //     boost::filesystem::create_directory(path);
    //     }
    //     std::ofstream fout_pose(path_pose, std::ios::app);
        
    //     // 构建4x4变换矩阵
    //     Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    //     transform.block<3,3>(0,0) = rotation;
    //     transform.block<3,1>(0,3) = translation;
    //     fout_pose << stamp_str << " " << transform(0, 0) << " "
    //             << transform(0, 1) << " " << transform(0, 2) << " "
    //             << transform(0, 3) << " " << transform(1, 0) << " "
    //             << transform(1, 1) << " " << transform(1, 2) << " "
    //             << transform(1, 3) << " " << transform(2, 0) << " "
    //             << transform(2, 1) << " " << transform(2, 2) << " "
    //             << transform(2,3) 
    //             //<< " 0.0 0.0 0.0 1.0" 
    //             << std::endl;

        
    // }

    // void save_pointcloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg, const std_msgs::msg::Header::_stamp_type& stamp) {
    //     pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    //     pcl::fromROSMsg(*cloud_msg, *cloud);

    //     // std::string filename = "cloud_" + std::to_string(stamp.sec) + "_" + std::to_string(stamp.nanosec) + ".pcd";
    //     // pcl::io::savePCDFileASCII(filename, *cloud);

    //     std::string path = "./pcd/";
    //     std::string stamp_str = std::to_string(rclcpp::Time(stamp).seconds());
    //     std::string file_name = stamp_str + ".pcd";
    //     std::string file_path = path + "top_center_lidar/";
    //     if (!boost::filesystem::exists(file_path)) {
    //         boost::filesystem::create_directory(file_path);
    //     }
    //     // pcl::transformPointCloud(*cloud_msg, *cloud_msg, T_lidar.matrix());
    //     pcl::io::savePCDFileBinary(file_path + file_name, *cloud);
    // }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OnlineCalib>());
    rclcpp::shutdown();
    return 0;
}
