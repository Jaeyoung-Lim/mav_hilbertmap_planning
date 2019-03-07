//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "hilbert_mappublisher/hilbert_mappublisher.h"

//Constructor
HilbertMapPublisher::HilbertMapPublisher(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private),
  mapcenter_(Eigen::Vector3d::Zero()){

    cmdloop_timer_ = nh_.createTimer(ros::Duration(0.1), &HilbertMapPublisher::cmdloopCallback, this); // Define timer for constant loop rate

    mapcenterPub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mappublisher/pose", 1);
    pointcloudPub_ = nh_.advertise<sensor_msgs::PointCloud2>("/mappublisher/pointcloud", 2);

    nh_.param<int>("/hilbert_mappublisher/width", map_width_, 1); //[m]
    nh_.param<double>("/hilbert_mappublisher/resolution", map_resolution_, 0.1); //[m/cells]
}

HilbertMapPublisher::~HilbertMapPublisher() {
  //Destructor
}

void HilbertMapPublisher::cmdloopCallback(const ros::TimerEvent& event) {

    pubMapCenter();
    pubPointCloud();

    ros::spinOnce();
}

void HilbertMapPublisher::pubMapCenter() {

    geometry_msgs::PoseStamped mapcenterpose_msg;

    mapcenterpose_msg.header.stamp = ros::Time::now();
    mapcenterpose_msg.header.frame_id = "world";
    mapcenterpose_msg.pose.position.x = mapcenter_(0);
    mapcenterpose_msg.pose.position.y = mapcenter_(1);
    mapcenterpose_msg.pose.position.z = mapcenter_(2);
    mapcenterpose_msg.pose.orientation.w = 1.0;
    mapcenterpose_msg.pose.orientation.x = 0.0;
    mapcenterpose_msg.pose.orientation.y = 0.0;
    mapcenterpose_msg.pose.orientation.z = 0.0;

    mapcenterPub_.publish(mapcenterpose_msg);
}

void HilbertMapPublisher::pubPointCloud(){

    sensor_msgs::PointCloud2 pointcloud_msg;
    pcl::PointCloud<pcl::PointXYZI> pointCloud;
    int cellmap_width;
    Eigen::Vector3d origin;
    origin << 0.5 * map_width_, 0.5 * map_width_, 0.5 * map_width_;

    cellmap_width = int(map_width_ / map_resolution_);
    //Encode pointcloud data
    for(int i = 0; i < cellmap_width; i ++) {
        for (int j = 0; j < cellmap_width; j++) {
            for (int k = 0; k < cellmap_width; k++) {
                pcl::PointXYZI point;

                point.x = i * map_resolution_ - origin(0);
                point.y = j * map_resolution_ - origin(1);
                point.z = k * map_resolution_ - origin(2);
                point.intensity = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z)/map_width_;

                pointCloud.points.push_back(point);
            }
        }
    }
    pcl::toROSMsg(pointCloud, pointcloud_msg);

    pointcloud_msg.header.stamp = ros::Time::now();
    pointcloud_msg.header.frame_id = "world";
    pointcloudPub_.publish(pointcloud_msg);
}