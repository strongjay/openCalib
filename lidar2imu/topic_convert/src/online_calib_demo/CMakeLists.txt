cmake_minimum_required(VERSION 3.5)
project(onlinecalib)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geodesy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(GeographicLib REQUIRED)
# 设置自定义Sophus路径
set(PROJECT_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(SOPHUS_SOURCE_DIR "${PROJECT_ROOT_DIR}/../../Sophus-1.22.4")
set(SOPHUS_BUILD_DIR "${SOPHUS_SOURCE_DIR}/build")

# 添加Sophus为子项目
add_subdirectory(${SOPHUS_SOURCE_DIR} ${SOPHUS_BUILD_DIR})

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${SOPHUS_SOURCE_DIR}      # 源码目录包含头文件
  ${PCL_INCLUDE_DIRS}  # 添加PCL头文件路径
  ${geodesy_INCLUDE_DIRS}
)

add_executable(onlinecalib 
  src/online_calib.cpp
)

target_link_libraries(onlinecalib
  # rclcpp::rclcpp
  # sensor_msgs::sensor_msgs
  # nav_msgs::nav_msgs
  # message_filters::message_filters
  # pcl_conversions::pcl_conversions
  Eigen3::Eigen
  Sophus::Sophus            # 链接编译生成的库
  Boost::filesystem
  ${PCL_LIBRARIES}
)

ament_target_dependencies(onlinecalib
  rclcpp
  sensor_msgs
  nav_msgs
  message_filters
  pcl_conversions
)
install(TARGETS onlinecalib
  DESTINATION lib/${PROJECT_NAME})


add_executable(odomPub 
  src/odomPub.cpp
)

# target_link_libraries(odomPub
#   Eigen3::Eigen
#   Sophus::Sophus            # 链接编译生成的库
#   Boost::filesystem
#   ${PCL_LIBRARIES}
# )

ament_target_dependencies(odomPub
  rclcpp
  tf2_ros
  sensor_msgs
  nav_msgs
  geometry_msgs
  geographic_msgs
  geodesy
)
install(TARGETS odomPub
  DESTINATION lib/${PROJECT_NAME})


add_executable(odomCalculate 
  src/odomCalculate.cpp
)

# target_link_libraries(odomPub
#   Eigen3::Eigen
#   Sophus::Sophus            # 链接编译生成的库
#   Boost::filesystem
#   ${PCL_LIBRARIES}
# )

ament_target_dependencies(odomCalculate
  rclcpp
  tf2_ros
  sensor_msgs
  nav_msgs
  geometry_msgs
  geographic_msgs
  geodesy
)
install(TARGETS odomCalculate
  DESTINATION lib/${PROJECT_NAME})
  
# sudo apt-get install libgeographic-dev
add_executable(dataCollector 
  src/dataCollector.cpp
)

target_link_libraries(dataCollector
  # rclcpp::rclcpp
  # sensor_msgs::sensor_msgs
  # nav_msgs::nav_msgs
  # message_filters::message_filters
  # pcl_conversions::pcl_conversions
  Eigen3::Eigen
  Sophus::Sophus            # 链接编译生成的库
  Boost::filesystem
  ${PCL_LIBRARIES}
  GeographicLib::GeographicLib 
)

ament_target_dependencies(dataCollector
  rclcpp
  sensor_msgs
  nav_msgs
  message_filters
  pcl_conversions
  geometry_msgs
  geographic_msgs
  geodesy
)
install(TARGETS dataCollector
  DESTINATION lib/${PROJECT_NAME})

ament_package()