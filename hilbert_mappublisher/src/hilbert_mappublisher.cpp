//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "hilbert_mappublisher/hilbert_mappublisher.h"

//Constructor
hilbertMapPublisher::hilbertMapPublisher(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private),
  mapcenter_(Eigen::Vector3d::Zero()){

    cmdloop_timer_ = nh_.createTimer(ros::Duration(0.1), &hilbertMapPublisher::cmdloopCallback, this); // Define timer for constant loop rate

    mapcenterPub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mappublisher/pose", 1);
    pointcloudPub_ = nh_.advertise<sensor_msgs::PointCloud2>("/mappublisher/pointcloud", 2);

    nh_.param<int>("/hilbert_mappublisher/width", map_width_, 10);
    nh_.param<double>("/hilbert_mappublisher/resolution", map_resolution_, 0.1);
}

hilbertMapPublisher::~hilbertMapPublisher() {
  //Destructor
}

void hilbertMapPublisher::cmdloopCallback(const ros::TimerEvent& event) {

    pubMapCenter();
    pubPointCloud();

    ros::spinOnce();
}

void hilbertMapPublisher::pubMapCenter() {

    geometry_msgs::PoseStamped mapcenterpose_msg;

    mapcenterpose_msg.header.stamp = ros::Time::now();
    mapcenterpose_msg.header.frame_id = "world";
    mapcenterpose_msg.pose.position.x = mapcenter_(0);
    mapcenterpose_msg.pose.position.y = mapcenter_(1);
    mapcenterpose_msg.pose.position.z = mapcenter_(2);
    mapcenterpose_msg.pose.orientation.w = 1.0;
    mapcenterpose_msg.pose.orientation.w = 0.0;
    mapcenterpose_msg.pose.orientation.w = 0.0;
    mapcenterpose_msg.pose.orientation.w = 0.0;

    mapcenterPub_.publish(mapcenterpose_msg);
}

void hilbertMapPublisher::pubPointCloud(){

    sensor_msgs::PointCloud2 pointcloud_msg;
    pcl::PointCloud<pcl::PointXYZI> pointCloud;
    //Encode pointcloud data
    for(int i = 0; i < map_width_; i ++) {
        for (int j = 0; j < map_width_; j++) {
            for (int k = 0; k < map_width_; k++) {
                pcl::PointXYZI point;

                point.x = i * map_resolution_ - 0.5 * map_width_ * map_resolution_;
                point.y = j * map_resolution_ - 0.5 * map_width_ * map_resolution_;
                point.z = k * map_resolution_ - 0.5 * map_width_ * map_resolution_;
                point.intensity = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z)/(map_width_*map_resolution_);

                pointCloud.points.push_back(point);
            }
        }
    }
    pcl::toROSMsg(pointCloud, pointcloud_msg);

    pointcloud_msg.header.stamp = ros::Time::now();
    pointcloud_msg.header.frame_id = "world";
    pointcloudPub_.publish(pointcloud_msg);
}