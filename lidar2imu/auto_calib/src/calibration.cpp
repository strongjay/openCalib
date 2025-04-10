/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 */

#include "calibration.hpp"
#include "BALM.hpp"
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "gen_BALM_feature.hpp"
#include "logging.hpp"

Calibrator::Calibrator(){

};

Calibrator::~Calibrator(){

};

/* 根据txt文件解析：对应时间戳的pcd文件名和位姿矩阵  */
void Calibrator::LoadTimeAndPoes(const std::string &filename,
                                 const Eigen::Matrix4d &Tl2i,
                                 std::vector<std::string> &lidarTimes,
                                 std::vector<Eigen::Matrix4d> &lidarPoses) {
  /* 解析雷达位姿文件 */
  std::ifstream file(filename);
  if (!file.is_open()) {
    std::cout << "ERROR--->>> cannot open: " << filename << std::endl;
    exit(1);
  }
  /* 读取包含时间戳和初始位姿的里程计数据 */
  double max_x, max_y, max_z, min_x, min_y, min_z; // 位姿坐标极值
  max_x = max_y = max_z = -INT_MAX;
  min_x = min_y = min_z = INT_MAX;
  std::string line; // 逐行读取
  while (getline(file, line)) {
    std::stringstream ss(line);
    std::string timeStr;
    ss >> timeStr; // 提取时间戳（第一个字段）
    lidar_files_.emplace_back(timeStr); // 存储对应时间戳的雷达数据

    Eigen::Matrix4d Ti = Eigen::Matrix4d::Identity(); // 位姿矩阵
    ss  >> Ti(0, 0) >> Ti(0, 1) >> Ti(0, 2) >> Ti(0, 3) 
        >> Ti(1, 0) >> Ti(1, 1) >> Ti(1, 2) >> Ti(1, 3) 
        >> Ti(2, 0) >> Ti(2, 1) >> Ti(2, 2) >> Ti(2, 3);
    Ti *= Tl2i; // 将IMU坐标转换到雷达坐标系
    max_x = std::max(max_x, Ti(0, 3));
    max_y = std::max(max_y, Ti(1, 3));
    max_z = std::max(max_z, Ti(2, 3));
    min_x = std::min(min_x, Ti(0, 3));
    min_y = std::min(min_y, Ti(1, 3));
    min_z = std::min(min_z, Ti(2, 3));
    lidarPoses.emplace_back(Ti); // 存储对应时间戳的雷达位姿
  }
  file.close();
}

/* 计算位姿变换矩阵 */
Eigen::Matrix4d Calibrator::GetDeltaTrans(double R[3], double t[3]) {
  Eigen::Matrix3d deltaR;
  double mat[9];
  // ceres::EulerAnglesToRotationMatrix(R, mat);
  ceres::AngleAxisToRotationMatrix(R, mat);
  deltaR << mat[0], mat[3], mat[6], mat[1], mat[4], mat[7], mat[2], mat[5],
      mat[8];
  // auto deltaR = Eigen::Matrix3d(
  //     Eigen::AngleAxisd(R[2], Eigen::Vector3d::UnitZ()) *
  //     Eigen::AngleAxisd(R[1], Eigen::Vector3d::UnitY()) *
  //     Eigen::AngleAxisd(R[0], Eigen::Vector3d::UnitX()));
  Eigen::Matrix4d deltaT = Eigen::Matrix4d::Identity();
  deltaT.block<3, 3>(0, 0) = deltaR;
  deltaT(0, 3) = t[0];
  deltaT(1, 3) = t[1];
  deltaT(2, 3) = t[2];
  return deltaT;
}

/* 主校准算法 */
void Calibrator::Calibration(const std::string lidar_path,
                             const std::string odom_path,
                             const Eigen::Matrix4d init_Tl2i) {
  lidar_path_ = lidar_path;
  auto time_begin = std::chrono::steady_clock::now();
  int turn = 20; // 设置最大迭代次数
  int window = 10; // 设置滑动窗口大小
  //   Eigen::Matrix4d init_Tl2i = Eigen::Matrix4d::Identity();
  Eigen::Matrix<double, 6, 1> last_deltaT; // 存储上一次的位姿变换
  /* 获取点云和位姿信息 */
  LoadTimeAndPoes(odom_path, init_Tl2i, lidar_files_, lidar_poses_);
  // 输出初始变换矩阵
  std::cout << "Initial Tl2i matrix:\n" << init_Tl2i << std::endl;
  // 输出前几帧的imu_T矩阵
  for (size_t i = 0; i < 5; i++) {
      std::cout << "Frame " << i << " imu_T:\n" << lidar_poses_[i] << std::endl;
  }
  /* 滑动窗口参数配置 */
  std::vector<int> frm_start_box; // 起始帧容器
  std::vector<int> frm_step_box; // 步长容器
  std::vector<int> frm_num_box; // 窗口数量容器
  int upper_bound = std::min(int(lidar_files_.size()), 1000); // 限制最大处理帧数
  int start_step = (upper_bound / 2) / turn_ - 1; // 计算起始步长
  /* 生成多组窗口配置 */
  for (int i = 0; i < turn_; i++) {
    int a = upper_bound / 2 - i * start_step - 1; // 动态计算起始帧
    frm_start_box.push_back(a);
    frm_step_box.push_back((upper_bound - a) / window_ - 1); // 步长递减
    frm_num_box.push_back(window_); // 固定窗口数量
  }
  /* 初始化位姿修正量（RPY角与平移） */
  double deltaRPY[3] = {0, 0, 0};
  double deltaT[3] = {0, 0, 0};
  /* 主优化循环 */
  for (int i = 0; i < frm_start_box.size(); i++) {
    std::cout << "\n==>ROUND " << i << std::endl;
    // 获取当前窗口参数
    int step = frm_step_box[i];
    int start = frm_start_box[i];
    int frmnum = frm_num_box[i];
    // 创建体素哈希表（平面特征与角点特征）
    std::unordered_map<VOXEL_LOC, OCTO_TREE *> surf_map, corn_map;
    OCTO_TREE::imu_transmat.clear();// 清空IMU位姿缓存
    // 生成当前修正变换矩阵
    Eigen::Matrix4d deltaTrans = GetDeltaTrans(deltaRPY, deltaT);
    OCTO_TREE::voxel_windowsize = frmnum; // 设置体素窗口大小
    /* 处理窗口内各帧数据 */
    int window_size = frmnum;
    for (size_t frmIdx = 0; frmIdx < frmnum; frmIdx++) {
      int real_frmIdx = start + frmIdx * step; // 计算实际帧索引
      /* 加载点云数据 */
      std::string lidar_file_name =
          lidar_path + lidar_files_[real_frmIdx] + ".pcd";
      pcl::PointCloud<LidarPointXYZIRT>::Ptr cloud(
          new pcl::PointCloud<LidarPointXYZIRT>);
      if (pcl::io::loadPCDFile(lidar_file_name, *cloud) < 0) {
        std::cout << "cannot open pcd_file: " << lidar_file_name << "\n";
        exit(1);
      }
      std::cout << " open pcd_file: " << lidar_file_name << "\n";
      /* 特征提取（平面/角点） */
      pcl::PointCloud<pcl::PointXYZI>::Ptr pl_corn(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::PointCloud<pcl::PointXYZI>::Ptr pl_surf(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::PointCloud<pcl::PointXYZI>::Ptr pl_surf_sharp(new pcl::PointCloud<pcl::PointXYZI>);
      genPcdFeature(cloud, pl_surf, pl_surf_sharp, pl_corn);
      std::cout << "Frame " << real_frmIdx 
          << ": Surf points = " << pl_surf->size()
          << ", Sharp surf points = " << pl_surf_sharp->size()
          << ", Corn points = " << pl_corn->size() << std::endl;
      // 应用当前修正后的位姿
      Eigen::Matrix4d imu_T = lidar_poses_[real_frmIdx];
      Eigen::Matrix4d refined_T = imu_T * deltaTrans;
      std::cout << "Refined_T for frame " << real_frmIdx << ":\n" << refined_T << std::endl;

      // 检查第一个点的变换后坐标
      if (!pl_surf_sharp->empty()) {
          Eigen::Vector3d pt(pl_surf_sharp->points[0].x, 
                            pl_surf_sharp->points[0].y, 
                            pl_surf_sharp->points[0].z);
          Eigen::Vector3d pt_transformed = (refined_T.block<3,3>(0,0)) * pt + refined_T.block<3,1>(0,3);
          std::cout << "Transformed point: " << pt_transformed.transpose() << std::endl;
      }

      OCTO_TREE::imu_transmat.push_back(imu_T);
      /* 体素地图构建（分阶段策略）*/
      if (i < turn / 2) { // 前半轮次使用锐利平面特征
        // cut_voxel(surf_map, pl_surf_sharp, refined_T, 0, frmIdx,window_size + 5);
        cut_voxel(surf_map, pl_surf, refined_T, 0, frmIdx, window_size + 5);
      } else {            // 后半轮次使用普通平面特征
        cut_voxel(surf_map, pl_surf, refined_T, 0, frmIdx, window_size + 5);
      }
      // if (i > turn / 2)
      //     cut_voxel(corn_map, pl_corn, refined_T, 1, frmIdx, window_size +
      //     5);

      /* 体素递归分割（构建八叉树）*/
      for (auto iter = surf_map.begin(); iter != surf_map.end(); ++iter) {
        if (iter->second->is2opt) // Sliding window of root voxel should
                                  // have points
        {
          iter->second->root_centors.clear();
          iter->second->recut(0, frmIdx, iter->second->root_centors);
        }
      }

      for (auto iter = corn_map.begin(); iter != corn_map.end(); ++iter) {
        if (iter->second->is2opt) {
          iter->second->root_centors.clear();
          iter->second->recut(0, frmIdx, iter->second->root_centors);
        }
      }
    }
    // display
    displayVoxelMap(surf_map);
    /* 优化位姿修正量 */
    if (i < turn / 2) {   // 前半轮次使用更高迭代次数
      optimizeDeltaTrans(surf_map, corn_map, 4, deltaRPY, deltaT);
    } else {              // 后半轮次减少迭代次数
      optimizeDeltaTrans(surf_map, corn_map, 2, deltaRPY, deltaT);
    }
    // 打印当前优化结果
    std::cout << "delta rpy: " << deltaRPY[0] / degree_2_radian << " "
              << deltaRPY[1] / degree_2_radian << " "
              << deltaRPY[2] / degree_2_radian << std::endl;
    std::cout << "delta T: " << deltaT[0] << " " << deltaT[1] << " "
              << deltaT[2] << std::endl;

    /* 清理体素树内存 */
    for (auto iter = corn_map.begin(); iter != corn_map.end(); ++iter) {
      clear_tree(iter->second);
    }
    for (auto iter = surf_map.begin(); iter != surf_map.end(); ++iter) {
      clear_tree(iter->second);
    }
    std::cout << "Round Finish!\n";
  }
  /* 保存最终标定结果 */
  double bestVal[6];
  bestVal[0] = deltaRPY[0];
  bestVal[1] = deltaRPY[1];
  bestVal[2] = deltaRPY[2];
  bestVal[3] = deltaT[0];
  bestVal[4] = deltaT[1];
  bestVal[5] = deltaT[2];
  // 计算总耗时
  auto time_end = std::chrono::steady_clock::now();
  std::cout << "calib cost "
            << std::chrono::duration<double>(time_end - time_begin).count()
            << "s" << std::endl;
  // 生成最终变换矩阵
  std::string refine_calib_file = "./refined_calib_imu_to_lidar.txt";
  Eigen::Matrix4d deltaTrans = Eigen::Matrix4d::Identity();
  // SaveStitching(deltaTrans,"before.pcd");
  deltaTrans = GetDeltaTrans(deltaRPY, deltaT);
  // SaveStitching(deltaTrans,"after.pcd");
  std::cout << "delta T is:" << std::endl;
  std::cout << deltaTrans << std::endl;
  auto refined_Tl2i = init_Tl2i * deltaTrans;
  auto refined_Ti2l = refined_Tl2i.inverse().eval();
  std::cout << "refined T(imu 2 lidar): " << std::endl; // 输出imu到lidar的变换
  std::cout << refined_Ti2l << std::endl;
  // 写入文件
  std::ofstream fCalib(refine_calib_file);
  if (!fCalib.is_open()) {
    std::cerr << "open file " << refine_calib_file << "failed." << std::endl;
    // return 1;
  }

  fCalib << "refined calib:" << std::endl;
  fCalib << "R: " << refined_Ti2l(0, 0) << " " << refined_Ti2l(0, 1) << " "
         << refined_Ti2l(0, 2) << " " << refined_Ti2l(1, 0) << " "
         << refined_Ti2l(1, 1) << " " << refined_Ti2l(1, 2) << " "
         << refined_Ti2l(2, 0) << " " << refined_Ti2l(2, 1) << " "
         << refined_Ti2l(2, 2) << std::endl;
  fCalib << "t: " << refined_Ti2l(0, 3) << " " << refined_Ti2l(1, 3) << " "
         << refined_Ti2l(2, 3) << std::endl;
  fCalib << "deltaTrans:" << std::endl;
  fCalib << deltaTrans << std::endl;
  fCalib << "delta roll, pitch, yaw, tx, ty, tz:" << std::endl;
  fCalib << bestVal[0] << " " << bestVal[1] << " " << bestVal[2] << " "
         << bestVal[3] << " " << bestVal[4] << " " << bestVal[5] << std::endl;
  fCalib << "delta roll, pitch, yaw, tx, ty, tz from begin:" << std::endl;
  fCalib << bestVal[0] + last_deltaT[0] << " " << bestVal[1] + last_deltaT[1]
         << " " << bestVal[2] + last_deltaT[2] << " "
         << bestVal[3] + last_deltaT[3] << " " << bestVal[4] + last_deltaT[4]
         << " " << bestVal[5] + last_deltaT[5] << std::endl;
  std::cout << "save refined calib to " << refine_calib_file << std::endl;
}

/* 结果保存 */
void Calibrator::SaveStitching(const Eigen::Matrix4d transform,
                               const std::string pcd_name) {

  pcl::PointCloud<pcl::PointXYZI>::Ptr all_cloud(
      new pcl::PointCloud<pcl::PointXYZI>());
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZI>::Ptr all_octree(
      new pcl::octree::OctreePointCloudSearch<pcl::PointXYZI>(0.3));

  all_octree->setInputCloud(all_cloud);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  for (size_t i = 0; i < lidar_files_.size(); i++) {
    std::string lidar_file_name = lidar_path_ + lidar_files_[i] + ".pcd";
    if (pcl::io::loadPCDFile(lidar_file_name, *cloud) < 0) {
      LOGW("can not open %s", lidar_file_name);
      return;
    }
    Eigen::Matrix4d T = lidar_poses_[i] * transform;
    for (const auto &src_pt : cloud->points) {
      if (!std::isfinite(src_pt.x) || !std::isfinite(src_pt.y) ||
          !std::isfinite(src_pt.z))
        continue;
      Eigen::Vector3d p(src_pt.x, src_pt.y, src_pt.z);
      Eigen::Vector3d p_res;
      p_res = T.block<3, 3>(0, 0) * p + T.block<3, 1>(0, 3);
      pcl::PointXYZI dst_pt;
      dst_pt.x = p_res(0);
      dst_pt.y = p_res(1);
      dst_pt.z = p_res(2);
      dst_pt.intensity = src_pt.intensity;
      if (!all_octree->isVoxelOccupiedAtPoint(dst_pt)) {
        all_octree->addPointToCloud(dst_pt, all_cloud);
      }
    }
  }
  pcl::io::savePCDFileASCII(pcd_name, *all_cloud);
  all_cloud->clear();
  all_octree->deleteTree();
}