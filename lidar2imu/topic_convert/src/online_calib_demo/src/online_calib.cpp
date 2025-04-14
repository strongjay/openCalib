#include "online_calib.h"

namespace localization {
namespace onlinecalib {

OnlineCalib::OnlineCalib()
    : Node("onlinecalib"), sync_policy_(SyncPolicy(100)) {
  this->declare_parameter<std::string>("lidar_topic", "/my_rslidar_points");
  this->declare_parameter<std::string>("imu_topic", "/my_odometry");
  this->get_parameter("lidar_topic", lidar_topic_);
  this->get_parameter("imu_topic", imu_topic_);
  RCLCPP_INFO(this->get_logger(), "lidar_topic: %s", lidar_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "imu_topic: %s", imu_topic_.c_str());

  is_calib_init_ = false;
  lid_front_left_sub_.subscribe(this, lidar_topic_);
  gnss_odom_sub_.subscribe(this, imu_topic_);
  sync_policy_.connectInput(lid_front_left_sub_, gnss_odom_sub_);
  sync_policy_.registerCallback(std::bind(&OnlineCalib::TimeSyncCallback, this,
                                          std::placeholders::_1,
                                          std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "Finish initialize");
  timer_ = this->create_wall_timer(std::chrono::milliseconds(20), 
                                   std::bind(&OnlineCalib::timer_callback, this));
}

void OnlineCalib::TimeSyncCallback( const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg,
                                    const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {
  // RCLCPP_INFO(this->get_logger(), "TimeSyncCallback");
  pcl::PointCloud<LidarPointXYZI>::Ptr cloud(new pcl::PointCloud<LidarPointXYZI>);
  cloud->clear();
  for (uint32_t i = 0; i < cloud_msg->width * cloud_msg->height; ++i) {
    LidarPointXYZI tmp_cloud;
    float x = 0.0, y = 0.0, z = 0.0;
    uint8_t r = 0, g = 0, b = 0, a = 0;

    memcpy(&tmp_cloud.x, &cloud_msg->data[i * cloud_msg->point_step + cloud_msg->fields[0].offset], sizeof(float));
    memcpy(&tmp_cloud.y, &cloud_msg->data[i * cloud_msg->point_step + cloud_msg->fields[1].offset], sizeof(float));
    memcpy(&tmp_cloud.z, &cloud_msg->data[i * cloud_msg->point_step + cloud_msg->fields[2].offset], sizeof(float));
    memcpy(&tmp_cloud.intensity, &cloud_msg->data[i * cloud_msg->point_step + cloud_msg->fields[3].offset], sizeof(float));
    // memcpy(&tmp_cloud.ring, &cloud_msg->data[i * cloud_msg->point_step + cloud_msg->fields[4].offset], sizeof(uint16_t));
    // memcpy(&tmp_cloud.timestamp, &cloud_msg->data[i * cloud_msg->point_step + cloud_msg->fields[5].offset], sizeof(double));
    cloud->points.push_back(tmp_cloud);
  }
  if (lidar_queue_.size() < 100 && gnss_odom_queue_.size() < 100) {
    lidar_queue_.push(cloud);
    gnss_odom_queue_.push(odom_msg);
    tf2::Quaternion q(  // 提取四元数
        odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y,
        odom_msg->pose.pose.orientation.z,
        odom_msg->pose.pose.orientation.w
    );
    tf2::Matrix3x3 m(q); // 转换为欧拉角
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    roll *= (180.0/M_PI); // 弧度转度数
    pitch *= (180.0/M_PI);
    yaw *= (180.0/M_PI);
    std::cout << "odom_msg orientation RPY(deg): "
              << " " << roll 
              << " " << pitch
              << " " << yaw 
              << "\nodom_msg position xyz(m):"
              << " "<< odom_msg->pose.pose.position.x
              << " " << odom_msg->pose.pose.position.y
              << " " << odom_msg->pose.pose.position.z
              << std::endl;
  } else {
    lidar_queue_.pop();
    gnss_odom_queue_.pop();
    lidar_queue_.push(cloud);
    gnss_odom_queue_.push(odom_msg);
  }
  return;
}

void OnlineCalib::timer_callback() {
  if (!lidar_queue_.empty() && !gnss_odom_queue_.empty()) {
    auto cloud_msg = lidar_queue_.front();
    auto odom_msg = gnss_odom_queue_.front();
    gnss_odom_queue_.pop();
    lidar_queue_.pop();

    Eigen::Vector3d position = Eigen::Vector3d(odom_msg->pose.pose.position.x,
                                               odom_msg->pose.pose.position.y,
                                               odom_msg->pose.pose.position.z);
    Eigen::Quaterniond rotation = Eigen::Quaterniond(odom_msg->pose.pose.orientation.w,
                                                     odom_msg->pose.pose.orientation.x,
                                                     odom_msg->pose.pose.orientation.y,
                                                     odom_msg->pose.pose.orientation.z
                                                     );
    // 当前位姿
    Sophus::SE3d current_pose(rotation, position);

    if (is_calib_init_ == false) {
      is_calib_init_ = true;
      T_lidar_init = current_pose;
      // IMU LIDAR 坐标系对齐 
      // Eigen::Matrix3d rot_correction;
      // rot_correction = Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ());
      // T_lidar_init = Sophus::SE3d(rot_correction, Eigen::Vector3d::Zero()) * T_lidar_init;
    }
    // 得到从初始位姿到当前位姿的相对变换
    Sophus::SE3d T_lidar = T_lidar_init.inverse() * current_pose;

    std::string path = "./debug/pcd/";
    std::string path_pose = path + "pose.txt";
    std::string stamp_str =
        std::to_string(rclcpp::Time(odom_msg->header.stamp).seconds());
    if (!boost::filesystem::exists(path)) {
      boost::filesystem::create_directory(path);
    }
    std::ofstream fout_pose(path_pose, std::ios::app);
    // 提取旋转矩阵
    Eigen::Matrix3d pose_matrix3d =
        T_lidar.rotationMatrix();//so3().unit_quaternion().toRotationMatrix();
    fout_pose << stamp_str << " " << pose_matrix3d(0, 0) << " "
              << pose_matrix3d(0, 1) << " " << pose_matrix3d(0, 2) << " "
              << T_lidar.translation().x() << " " << pose_matrix3d(1, 0) << " "
              << pose_matrix3d(1, 1) << " " << pose_matrix3d(1, 2) << " "
              << T_lidar.translation().y() << " " << pose_matrix3d(2, 0) << " "
              << pose_matrix3d(2, 1) << " " << pose_matrix3d(2, 2) << " "
              << T_lidar.translation().z() 
              //<< " 0.0 0.0 0.0 1.0" 
              << std::endl;
    std::cout<< "full matrix: \n" << T_lidar.matrix() << std::endl;
    std::string file_name = stamp_str + ".pcd";
    std::string file_path = path + "top_center_lidar/";
    if (!boost::filesystem::exists(file_path)) {
      boost::filesystem::create_directory(file_path);
    }
    // pcl::transformPointCloud(*cloud_msg, *cloud_msg, T_lidar.matrix());
    pcl::io::savePCDFileBinary(file_path + file_name, *cloud_msg);
  }
}
}  // namespace onlinecalib
}  // namespace localization

int main(int argc, char **argv) {
    std::filesystem::path dir_path = "debug/pcd";

  try {
      // 创建多级目录
      bool created = std::filesystem::create_directories(dir_path);
      if (created) {
          std::cout << "目录创建成功: " << dir_path << std::endl;
      } else {
          std::cout << "目录已存在: " << dir_path << std::endl;
      }
  } catch (const std::filesystem::filesystem_error& e) {
      std::cerr << "错误: " << e.what() << std::endl;
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<localization::onlinecalib::OnlineCalib>());
  rclcpp::shutdown();
  return 0;
}

