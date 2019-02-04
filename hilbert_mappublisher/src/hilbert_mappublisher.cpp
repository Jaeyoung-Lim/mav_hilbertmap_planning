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
    pcl::PointXYZI point;

    //Encode pointcloud data
    point.x = 1024 * rand () / (RAND_MAX + 1.0f);
    point.y = 1024 * rand () / (RAND_MAX + 1.0f);
    point.z = 1.0;

    pointCloud.points.push_back(point);

    pcl::toROSMsg(pointCloud, pointcloud_msg);

    pointcloud_msg.header.stamp = ros::Time::now();
    pointcloud_msg.header.frame_id = "world";
    

    pointcloudPub_.publish(pointcloud_msg);

}