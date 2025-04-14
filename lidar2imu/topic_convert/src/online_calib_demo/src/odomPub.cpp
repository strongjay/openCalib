#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"  
#include "sensor_msgs/msg/imu.hpp"
#include "geographic_msgs/msg/geo_point.hpp"
#include "geodesy/utm.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geographic_msgs/msg/geo_point.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <tf2/LinearMath/Matrix3x3.h>

class GPSOdomPublisher : public rclcpp::Node
{
public:
    GPSOdomPublisher() : Node("gps_odom_publisher")
    {    
        // 参数设置
        this->declare_parameter<std::string>("gps_fix_topic", "/fix");
        this->declare_parameter<std::string>("imu_topic", "/imu");
        this->imu_topic_ = this->get_parameter("imu_topic").as_string();
        this->gps_fix_topic_ = this->get_parameter("gps_fix_topic").as_string();

        // 订阅GPS和IMU话题
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            this->get_parameter("gps_fix_topic").as_string(), 10,
            std::bind(&GPSOdomPublisher::gps_callback, this, std::placeholders::_1));
            
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            this->get_parameter("imu_topic").as_string(), 10,
            std::bind(&GPSOdomPublisher::imu_callback, this, std::placeholders::_1));

        //////////////////////////////////////////////////////////////////////////////////
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "rslidar_points", 10,
            std::bind(&GPSOdomPublisher::pointcloud_callback, this, std::placeholders::_1));
        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("my_rslidar_points", 10);
        /////////////////////////////////////////////////////////////////////////////////////
        
        RCLCPP_INFO(this->get_logger(), "subscribe gps_fix topic : %s, imu topic: %s", gps_fix_topic_.c_str(), imu_topic_.c_str());
        
        // 初始化Odometry发布器
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/my_odometry", 10);
        
        // 初始化成员变量
        gps_init_ = false;
        init_x_ = 0.0;
        init_y_ = 0.0;
        init_z_ = 0.0;
        current_orientation_ = tf2::Quaternion::getIdentity();
        RCLCPP_INFO(this->get_logger(), "Initialization done!!!");
        total_distance_ = 0.0;
        first_position_ = true;
    }

private:

    /////////////////////////////////////////////////////////////////////////////////////////////
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        auto new_msg = *msg;  // Make a copy of the message
        new_msg.header.stamp = this->now();  // Update the timestamp
        
        // Optionally update frame_id if needed
        // new_msg.header.frame_id = "your_frame_id";
        
        pointcloud_pub_->publish(new_msg);
    }
    ////////////////////////////////////////////////////////////////////////////////////////
    // 将经纬度转换为UTM坐标的函数
    void lon_lat_to_utm(double lon, double lat, double &utm_x, double &utm_y)
    {
        geographic_msgs::msg::GeoPoint gp;
        gp.latitude = lat;
        gp.longitude = lon;
        geodesy::UTMPoint pt(gp);
        utm_x = pt.easting;
        utm_y = pt.northing;
    }

    // IMU数据回调
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        if(!imu_init_){
            imu_init_ = true;
            init_orientation_.setX(msg->orientation.x);
            init_orientation_.setY(msg->orientation.y);
            init_orientation_.setZ(msg->orientation.z);
            init_orientation_.setW(msg->orientation.w);
            return;
        }
        // 更新当前姿态
        current_orientation_.setX(msg->orientation.x);
        current_orientation_.setY(msg->orientation.y);
        current_orientation_.setZ(msg->orientation.z);
        current_orientation_.setW(msg->orientation.w);
    }

    // GPS数据回调函数
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        // 检查GPS数据是否有效
        if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
            RCLCPP_WARN(this->get_logger(), "No GPS fix");
            return;
        }

        // 如果尚未初始化，使用第一个GPS消息作为起始点
        if(!gps_init_ && msg->status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX)
        {
            gps_init_ = true;
            // 原点经纬度转UTM
            geographic_msgs::msg::GeoPoint gp_init;
            gp_init.latitude = msg->latitude;;
            gp_init.longitude = msg->longitude;
            gp_init.altitude = msg->altitude;
            geodesy::UTMPoint pt_init(gp_init);
            init_x_ = pt_init.easting;
            init_y_ = pt_init.northing;
            init_z_ = pt_init.altitude;
            RCLCPP_INFO(this->get_logger(), "Origin set at lat: %.6f, lon: %.6f, alt: %.6f", init_x_, init_y_, init_z_);
            return;
        }

        // 将当前GPS位置转换为UTM坐标
        geographic_msgs::msg::GeoPoint gp;
        gp.latitude = msg->latitude;
        gp.longitude = msg->longitude;
        gp.altitude = msg->altitude;
        geodesy::UTMPoint pt(gp);
        double fix_x = pt.easting - init_x_;
        double fix_y = pt.northing - init_y_;
        double fix_z = pt.altitude - init_z_;
        // 更新里程
        updateDistance();
        updateOdometry();
        
    }


    void updateDistance() {
        geometry_msgs::msg::Point current_pos;
        current_pos.x = fix_x;
        current_pos.y = fix_y;
        current_pos.z = fix_z;
        // 首次收到位置，不计算距离，只更新上一次位置
        if (first_position_) {
            last_pos_ = current_pos;
            first_position_ = false;
            return;
        }
        // 计算水平位移（忽略高度变化）
        double dx = current_pos.x - last_pos_.x;
        double dy = current_pos.y - last_pos_.y;
        double delta_distance = std::sqrt(dx * dx + dy * dy);
        // 累加总里程
        total_distance_ += delta_distance;
        last_pos_ = current_pos;
        std::cout<< "DIS: \t" << total_distance_ << std::endl << std::endl;
    }

    void updateOdometry(){
        // 构建Odometry消息
        auto gps_odom = nav_msgs::msg::Odometry();
        gps_odom.header.stamp = this->now();
        gps_odom.header.frame_id = "rslidar";
        gps_odom.child_frame_id = "gps";
        // 设置位置（相对原点）
        gps_odom.pose.pose.position.x = fix_x;
        gps_odom.pose.pose.position.y = fix_y;
        gps_odom.pose.pose.position.z = fix_z;
        // 设置方向（来自IMU）
        tf2::Quaternion q_rel = current_orientation_ * init_orientation_.inverse() ;
        q_rel.normalize();  // 确保单位化
        gps_odom.pose.pose.orientation.x = q_rel.x();
        gps_odom.pose.pose.orientation.y = q_rel.y();
        gps_odom.pose.pose.orientation.z = q_rel.z();
        gps_odom.pose.pose.orientation.w = q_rel.w();
        // 调试输出
        double roll, pitch, yaw;
        tf2::Matrix3x3(q_rel).getRPY(roll, pitch, yaw);
        std::cout << "POS: \t" << fix_x << " \t" << fix_y << " \t" << fix_z << " \t" << std::endl;
        std::cout << "ROT: \t" << roll*180/M_PI << " \t" << pitch*180/M_PI << " \t" << yaw*180/M_PI << " \t" << std::endl;

        
        // 发布Odometry消息
        odom_pub_->publish(gps_odom);
    }

    // ROS2订阅器和发布器
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    
    ///////////////////////////////////////////////////////////////////////////////////
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;

    //////////////////////////////////////////////////////////////////////////////////////
    // 成员变量
    bool imu_init_ = false;
    bool gps_init_ = false;
    double init_x_, init_y_, init_z_;
    tf2::Quaternion init_orientation_;
    tf2::Quaternion current_orientation_;
    std::string imu_topic_;
    std::string gps_fix_topic_;

    // 里程
    double fix_x = 0.0;
    double fix_y = 0.0;
    double fix_z = 0.0;
    double total_distance_ = 0.0;         // 总里程
    bool first_position_ = true;          // 首次位置标记
    geometry_msgs::msg::Point last_pos_;  // 上一次位置

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GPSOdomPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
