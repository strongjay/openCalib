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
        init_lat_ = 0.0;
        init_lon_ = 0.0;
        init_alt_ = 0.0;
        init_x_ = 0.0;
        init_y_ = 0.0;
        init_z_ = 0.0;
        current_orientation_ = tf2::Quaternion::getIdentity();
        RCLCPP_INFO(this->get_logger(), "Initialization done!!!");
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
            RCLCPP_INFO(this->get_logger(), "IMU init done!");
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
        // 初始化原点
        if(!gps_init_ && msg->status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX)
        {
            init_lat_ = msg->latitude;
            init_lon_ = msg->longitude;
            init_alt_ = msg->altitude;
            gps_init_ = true;
            
            // 原点经纬度转UTM
            geographic_msgs::msg::GeoPoint gp_init;
            gp_init.latitude = init_lat_;
            gp_init.longitude = init_lon_;
            gp_init.altitude = init_alt_;
            geodesy::UTMPoint pt_init(gp_init);
            init_x_ = pt_init.easting;
            init_y_ = pt_init.northing;
            init_z_ = pt_init.altitude;
            // // 设置局部笛卡尔坐标系原点参数
            // this->set_parameter(rclcpp::Parameter("origin_x", init_lat_));
            // this->set_parameter(rclcpp::Parameter("origin_y", init_lon_));
            // this->set_parameter(rclcpp::Parameter("origin_z", init_alt_));
            
            RCLCPP_INFO(this->get_logger(), "Origin set at lat: %.6f, lon: %.6f, alt: %.6f", init_lat_, init_lon_, init_alt_);
            return;
        }
        
        

        // 将当前GPS位置转换为UTM坐标
        geographic_msgs::msg::GeoPoint gp;
        gp.latitude = msg->latitude;
        gp.longitude = msg->longitude;
        gp.altitude = msg->altitude;
        geodesy::UTMPoint pt(gp);
        double fix_x = pt.easting;
        double fix_y = pt.northing;
        double fix_z = pt.altitude;
        
        // RCLCPP_INFO(this->get_logger(), "LocalCartesian position: %f, %f", fix_x - init_x_, fix_y - init_y_);

        // 构建Odometry消息
        auto gps_odom = nav_msgs::msg::Odometry();
        gps_odom.header.stamp = this->now();
        gps_odom.header.frame_id = "rslidar";
        gps_odom.child_frame_id = "gps";
        // 设置位置（相对原点）
        gps_odom.pose.pose.position.x = (fix_x - init_x_);
        gps_odom.pose.pose.position.y = (fix_y - init_y_);
        gps_odom.pose.pose.position.z = (fix_z - init_z_);
        // 设置方向（来自IMU）
        tf2::Quaternion q_rel = current_orientation_ * init_orientation_.inverse() ;
        q_rel.normalize();  // 确保单位化
        // 调试输出
        // double roll, pitch, yaw;
        // tf2::Matrix3x3(q_rel).getRPY(roll, pitch, yaw);
        // RCLCPP_INFO(this->get_logger(), "Corrected RPY: [%.2f°, %.2f°, %.2f°]", 
        //             roll*180/M_PI, pitch*180/M_PI, yaw*180/M_PI);
        gps_odom.pose.pose.orientation.x = q_rel.x();
        gps_odom.pose.pose.orientation.y = q_rel.y();
        gps_odom.pose.pose.orientation.z = q_rel.z();
        gps_odom.pose.pose.orientation.w = q_rel.w();


        
        // 发布Odometry消息
        odom_pub_->publish(gps_odom);
        std::cout<<"current position: "<<gps_odom.pose.pose.position.x<<" "<<gps_odom.pose.pose.position.y<<" "<<gps_odom.pose.pose.position.z<<std::endl;
        std::cout<<"current orientation: "<<gps_odom.pose.pose.orientation.x<<" "<<gps_odom.pose.pose.orientation.y<<" "<<gps_odom.pose.pose.orientation.z<<" "<<gps_odom.pose.pose.orientation.w<<std::endl;
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

    double init_lat_, init_lon_, init_alt_;
    double init_x_, init_y_, init_z_;
    tf2::Quaternion init_orientation_;
    tf2::Quaternion current_orientation_;
    std::string imu_topic_;
    std::string gps_fix_topic_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GPSOdomPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
