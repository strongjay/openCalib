#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include "chcnav/hcinspvatzcb.h"
#include <geodesy/utm.h>


class GPSOdomPublisher
{
    public:
        GPSOdomPublisher();  
        void gpsCallback(const chcnav::hcinspvatzcb::ConstPtr &msg);                // GPS回调函数
        void lonAndLat2UTM(double lon, double lat, double &UTMX, double &UTMY);     // 经纬度转UTM坐标

    private:
        ros::Publisher gpsOdomPub, gpsPathPub;      // 发布器：Odometry和Path
        ros::Subscriber subGPS, subHeading;         // 订阅器：GPS和航向
        ros::NodeHandle n, nhPrivate;               // ROS节点句柄
        bool init;                                  // 初始化标志

        double initLat, initLon, initAlt;           // 初始经纬度和高度
        double initX, initY;                        // 初始UTM坐标
        std::string gpsFixTopic, headingTopic;      // 订阅话题名称
        double heading;                             // 当前航向角
};

GPSOdomPublisher::GPSOdomPublisher():nhPrivate("~")
{    
    // 从参数服务器获取GPS话题名称，默认为"/chcnav/devpvt"
    nhPrivate.param("gpsFixTopicSimple", gpsFixTopic, std::string("/chcnav/devpvt"));
    ROS_INFO("subscribe gpsFixTopic : %s", gpsFixTopic.c_str());
    
    // 订阅GPS话题
    subGPS = n.subscribe("/chcnav/devpvt", 10, &GPSOdomPublisher::gpsCallback, this);
    // 初始化Odometry发布器
    gpsOdomPub = n.advertise<nav_msgs::Odometry>("/gps/odometry", 10);		
    // 初始化成员变量
    this->init = false;
    this->initLat = 0.0;
    this->initLon = 0.0;
    this->initAlt = 0.0;
    this->initX = 0.0;
    this->initY = 0.0;

    ROS_INFO("Initialization done!!!");
}

// 将经纬度转换为UTM坐标的函数
void GPSOdomPublisher::lonAndLat2UTM(double lon, double lat, double &UTMX, double &UTMY)
{
    geographic_msgs::GeoPoint gp;   // 创建地理坐标点
    gp.latitude = lat;              // 设置纬度
    gp.longitude = lon;             // 设置经度
    geodesy::UTMPoint pt(gp);       // 转换为UTM坐标
    UTMX = pt.easting;              // 输出UTM东向坐标
    UTMY = pt.northing;             // 输出UTM北向坐标
}

// GPS数据回调函数
void GPSOdomPublisher::gpsCallback(const chcnav::hcinspvatzcb::ConstPtr &msg)
{
    // ROS_INFO("gpsCallback=======");
    /*不进入固定解，不发布odom*/
    // if (msg->stat[1] == 4 || msg->stat[1] == 8)
    //     return;
    
    // 如果尚未初始化，使用第一个GPS消息作为起始点
    if(!this->init)
    {
        this->initLat = msg->latitude;      // 设置初始纬度
        this->initLon = msg->longitude;     // 设置初始经度
        this->initAlt = msg->altitude;      // 设置初始高度
        this->init = true;                  // 标记已初始化
        /* 原点经纬度转UTM */
        geographic_msgs::GeoPoint gpInit;
        gpInit.latitude = this->initLat;
        gpInit.longitude = this->initLon;
        geodesy::UTMPoint ptInit(gpInit);
        this->initX = ptInit.easting;       // 设置初始UTM X坐标
	    this->initY = ptInit.northing;      // 设置初始UTM Y坐标
        /* 设置局部笛卡尔坐标系原点 */
        ros::param::set("origin_x", this->initLat);
        ros::param::set("origin_y", this->initLon);
        ros::param::set("origin_z", this->initAlt);
        ROS_INFO("LocalCartesian init successfully!!! position: %f, %f, %f", this->initLat, this->initLon, this->initAlt);
        return;
    }
    
    ROS_INFO("lat, lon, alt: %.8f, %.8f, %.8f", msg->latitude, msg->longitude, msg->altitude);

    // 将当前GPS位置转换为UTM坐标
    geographic_msgs::GeoPoint gp;
    gp.latitude = msg->latitude;
    gp.longitude = msg->longitude;
    geodesy::UTMPoint pt(gp);
    double fixX = pt.easting;
    double fixY = pt.northing;
    double heading = msg->heading / 180.0 * M_PI;
    ROS_INFO("LocalCartesian position: %f, %f", fixX - initX, fixY - initY);

    nav_msgs::Odometry gpsOdom;
    gpsOdom.header.stamp = msg->header.stamp;       // 时间戳
    gpsOdom.header.frame_id = "map";                // 坐标系
    gpsOdom.pose.pose.position.x = fixX - initX;    // 相对初始点的X坐标
    gpsOdom.pose.pose.position.y = fixY - initY;    // 相对初始点的Y坐标
    gpsOdom.pose.pose.position.z = 0.0;             // Z坐标设为0
    gpsOdom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, heading); // 从欧拉角创建四元数（只使用偏航角）
    gpsOdomPub.publish(gpsOdom);
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "hcinspvatzcb_subscriber");
    ros::NodeHandle nh;

    GPSOdomPublisher gop;

    ros::spin();

    return 0;
}