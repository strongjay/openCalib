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
#include <Eigen/Dense>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
struct Piont
{
    Eigen::Vector3d pos;//位置
    Eigen::Matrix3d orien;//姿态 旋转矩阵表示
    Eigen::Vector3d w;//角速度
    Eigen::Vector3d v;//线速度
};

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
        // gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        //     this->get_parameter("gps_fix_topic").as_string(), 10,
        //     std::bind(&GPSOdomPublisher::gps_callback, this, std::placeholders::_1));
            
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
        
        Eigen::Vector3d zero(0, 0, 0);
        point.pos = zero;
        point.orien = Eigen::Matrix3d::Identity();
        point.v = zero;
        point.w = zero;
        firstT = true;
        RCLCPP_INFO(this->get_logger(), "Initialization done!!!");

        
    }

private:

    /////////////////////////////////////////////////////////////////////////////////////////////
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        auto new_msg = *msg;  // Make a copy of the message
        new_msg.header.stamp = this->now();  // Update the timestamp
        
        // Optionally update frame_id if needed
        new_msg.header.frame_id = "map";
        
        pointcloud_pub_->publish(new_msg);
    }


    // IMU数据回调
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg){
        rclcpp::Time msg_stamp = rclcpp::Time(msg->header.stamp);
        if (firstT) {
            deltaT = 0;
            setGravity(msg->linear_acceleration);//这里只是粗略的拿第一帧数据作为当前的重力即速度，可以变成用前几帧的数据估计出一个g
            firstT = false;
            time = msg_stamp;
        } else {
            if (time.seconds() != 0.0) { // 避免首次调用时 time 未初始化
                rclcpp::Duration duration = msg_stamp - time;
                deltaT = duration.seconds();

            }
            odom.header.stamp = this->now(); 
            odom.header.frame_id = "map";
            calcOrientation(msg->angular_velocity);//计算角度，四元数表示
            calcPosition(msg->linear_acceleration);//计算位置
            updateodom(point);
        }
        time = msg_stamp;

    }

    void setGravity(const geometry_msgs::msg::Vector3 &msg) {
        // gravity[0] = msg.x;
        // gravity[1] = msg.y;
        // gravity[2] = msg.z;
        Eigen::Vector3d acc_imu(msg.x, msg.y, msg.z);
        // 将初始加速度转换到世界坐标系（point.orien初始为Identity）
        gravity = point.orien * acc_imu; 
        // 重力在世界坐标系应近似为 [0, 0, 9.81]
        gravity[2] = 9.81;  // 强制z轴为重力
    }

    void calcOrientation(const geometry_msgs::msg::Vector3 &msg) {
        // 欧拉法积分角速度会导致误差累积
        // point.w << msg.x, msg.y, msg.z;
        // //基于旋转矩阵表示方法
        // Eigen::Matrix3d B;
        // B << 0, -msg.z * deltaT, msg.y * deltaT, 
        //     msg.z * deltaT, 0, -msg.x * deltaT,
        //     -msg.y * deltaT, msg.x * deltaT, 0;
        // //欧拉法
        // double sigma =
        //     std::sqrt(std::pow(msg.x, 2) + std::pow(msg.y, 2) + std::pow(msg.z, 2)) *
        //     deltaT;
        // // std::cout << "sigma: " << sigma << std::endl << Eigen::Matrix3d::Identity()
        // // + (std::sin(sigma) / sigma) * B << std::endl << pose.orien << std::endl;
        // //罗德里格斯公式
        // point.orien = point.orien *
        //             (Eigen::Matrix3d::Identity() + (std::sin(sigma) / sigma) * B -
        //                 ((1 - std::cos(sigma)) / std::pow(sigma, 2)) * B * B);

        // 使用 四元数姿态更新 减少误差
         Eigen::Quaterniond q_current(point.orien);
        // 角速度转四元数增量
        Eigen::Vector3d angular_vel(msg.x, msg.y, msg.z);
        Eigen::Quaterniond delta_q;
        double theta = angular_vel.norm() * deltaT;
        if (theta > 1e-6) {
            Eigen::Vector3d axis = angular_vel / angular_vel.norm();
            delta_q = Eigen::AngleAxisd(theta, axis);
        } else {
            delta_q = Eigen::Quaterniond::Identity();
        }
        
        // 更新姿态
        q_current = q_current * delta_q;
        point.orien = q_current.toRotationMatrix();
        std::cout << "orien: " << point.orien << std::endl;
    }

    void calcPosition(const geometry_msgs::msg::Vector3 &msg) {
        Eigen::Vector3d acc_l(msg.x, msg.y, msg.z);//imu坐标系下的加速度
        Eigen::Vector3d acc_g = point.orien * acc_l;//转化到里程计坐标系下的加速度
        // Eigen::Vector3d acc(msg.x - gravity[0], msg.y - gravity[1], msg.z -
        // gravity[2]);
        point.v = point.v + deltaT * (acc_g - gravity);//积分得到速度
        point.pos = point.pos + deltaT * point.v;//积分得到位置
        std::cout << "pos: " << point.pos << std::endl;
    }

    void updateodom(const Piont point) {
        // //位置
        // odom.pose.pose.position.x = point.pos(0);
        // odom.pose.pose.position.y = point.pos(1);
        // odom.pose.pose.position.z = point.pos(2);
        // //姿态 四元数
        // odom.pose.pose.orientation.x = (point.orien(2,1) - point.orien(1,2)) / 4; 
        // odom.pose.pose.orientation.y = (point.orien(0,2) - point.orien(2,0)) / 4;
        // odom.pose.pose.orientation.z = (point.orien(1,0) - point.orien(0,1)) / 4;
        // odom.pose.pose.orientation.w = std::sqrt(1 + point.orien(0,0) + point.orien(1,1) + point.orien(2,2)) / 2;
        // //线速度
        // odom.twist.twist.linear.x = point.v(0);
        // odom.twist.twist.linear.y = point.v(1);
        // odom.twist.twist.linear.z = point.v(2);

        // //角速度
        // odom.twist.twist.angular.x = point.w(0);
        // odom.twist.twist.angular.y = point.w(1);
        // odom.twist.twist.angular.z = point.w(2);
        // //发布里程计
        // odom_pub_->publish(odom);

        if(!firstP){
            point_first = point;
            firstP = true;
            return ;
        }
        odom.pose.pose.position.x = point.pos(0) - point_first.pos(0);  // point_y → vehicle_x
        odom.pose.pose.position.y = point.pos(1) - point_first.pos(1); // -point_x → vehicle_y
        odom.pose.pose.position.z = point.pos(2) - point_first.pos(2);  // z保持原样
        
        Eigen::Matrix3d R_rel = point_first.orien.inverse() * point.orien;
        Eigen::Quaterniond q(R_rel);
        q.normalize();

        // 设置四元数值
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        // 速度转换（同位置逻辑）
        Eigen::Vector3d v_initial = point_first.orien.inverse() * point.v;
        odom.twist.twist.linear.x = v_initial.x();//point.v(0);   // v_y → vehicle_vx
        odom.twist.twist.linear.y = v_initial.y();//point.v(1);  // -v_x → vehicle_vy
        odom.twist.twist.linear.z = v_initial.z();//point.v(2);

        // 角速度转换（需要同步坐标系旋转）
        Eigen::Vector3d w_initial = point_first.orien.inverse() * point.w;
        odom.twist.twist.angular.x = w_initial.x();//point.w(0);  // w_y → vehicle_wx
        odom.twist.twist.angular.y = w_initial.y();//point.w(1); // -w_x → vehicle_wy 
        odom.twist.twist.angular.z = w_initial.z();//point.w(2);

        odom_pub_->publish(odom);
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

    nav_msgs::msg::Odometry odom;
    rclcpp::Time time;
    Piont point;
    Piont point_first;
    bool firstP = false;
    Eigen::Vector3d gravity;
    double deltaT;
    bool firstT;
    // 在静止时采集零偏均值。
    Eigen::Vector3d angular_bias = Eigen::Vector3d::Zero();
    int calibration_count = 0;
    const int CALIBRATION_STEPS = 100;  // 静止校准帧数
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GPSOdomPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
