//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "hilbert_mappublisher/hilbert_mappublisher.h"

//Constructor
hilbertMapPublisher::hilbertMapPublisher(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private),
  mapcenter_(Eigen::Vector3d::Zero()){

    cmdloop_timer_ = nh_.createTimer(ros::Duration(0.01), &hilbertMapPublisher::cmdloopCallback, this); // Define timer for constant loop rate
}

hilbertMapPublisher::~hilbertMapPublisher() {
  //Destructor
}

void hilbertMapPublisher::cmdloopCallback(const ros::TimerEvent& event) {

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

    mapcenter_pub_.publish(mapcenterpose_msg);
}