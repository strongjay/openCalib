#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
import numpy as np
import builtin_interfaces
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovariance, TwistWithCovariance
from std_msgs.msg import Header
import time

from geometry_msgs.msg import Quaternion
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf2_geometry_msgs 


class FakeSensorPublisher(Node):
    def __init__(self):
        super().__init__('fake_sensor_publisher')
        
        # 初始化发布器
        self.pc_pub = self.create_publisher(PointCloud2, '/my_rslidar_points', 10)
        self.odom_pub = self.create_publisher(Odometry, '/my_odometry', 10)
        
        # 初始化状态变量
        self.x_position = 1000.0  # 累计平移量
        self.frame_id = 0      # 时间戳基准
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        # 创建1Hz定时器
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info("Fake sensor publisher started")

    def get_timestamp(self):
        """生成统一的时间戳"""
        now = self.get_clock().now().seconds_nanoseconds()
        return builtin_interfaces.msg.Time(sec=now[0], nanosec=now[1])

    def create_pointcloud(self):
        """生成随机点云数据"""
        num_points = 1000
        
        # 生成随机点坐标 (XYZ)
        points = np.random.rand(num_points, 3).astype(np.float32)
        
        # 构造PointCloud2消息
        msg = PointCloud2()
        msg.header = Header(
            stamp=self.get_timestamp(),
            frame_id='random'
        )
        
        # 设置点云字段描述
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='i', offset=12, datatype=PointField.FLOAT32, count=1)
        ]
        
        msg.height = 1
        msg.width = num_points
        msg.point_step = 12  # 每个点占12字节 (3个float32)
        msg.row_step = msg.point_step * num_points
        msg.data = points.tobytes()
        msg.is_dense = True
        
        return msg

    def create_odometry(self):
        """生成累加平移的里程计消息"""
        self.x_position += 1  # 每帧X轴增加0.1米
        pos = Point(x=self.x_position, y=0.0, z=0.0)

        self.roll =  np.deg2rad(0.0)
        self.pitch =  np.deg2rad(0.0)
        self.yaw += np.deg2rad(0.0)  # 每帧Y轴增加1度
        # 计算各轴的半角三角函数
        cr = np.cos(self.roll * 0.5)
        sr = np.sin(self.roll * 0.5)
        cp = np.cos(self.pitch * 0.5)
        sp = np.sin(self.pitch * 0.5)
        cy = np.cos(self.yaw * 0.5)
        sy = np.sin(self.yaw * 0.5)

        # Z-Y-X旋转顺序（yaw-pitch-roll）的四元数计算
        quaternion = Quaternion()
        quaternion.w = cr * cp * cy + sr * sp * sy
        quaternion.x = sr * cp * cy - cr * sp * sy
        quaternion.y = cr * sp * cy + sr * cp * sy
        quaternion.z = cr * cp * sy - sr * sp * cy

        # 四元数归一化（重要！）
        norm = np.sqrt(quaternion.w**2 + quaternion.x**2 + 
                    quaternion.y**2 + quaternion.z**2)
        quaternion.w /= norm
        quaternion.x /= norm
        quaternion.y /= norm
        quaternion.z /= norm

        msg = Odometry()
        msg.header = Header(
            stamp=self.get_timestamp(),
            frame_id='random'
        )
        
        # 设置姿态 (保持水平)
        

        msg.pose.pose = Pose(
            position=pos,
            orientation=quaternion  # 无旋转
        )
        
        return msg

    def timer_callback(self):
        """定时器回调函数"""
        pc_msg = self.create_pointcloud()
        odom_msg = self.create_odometry()
        
        self.pc_pub.publish(pc_msg)
        self.odom_pub.publish(odom_msg)
        
        self.get_logger().info(f"Published frame {self.frame_id}")
        self.frame_id += 1

def main(args=None):
    rclpy.init(args=args)
    node = FakeSensorPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down fake publisher")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
