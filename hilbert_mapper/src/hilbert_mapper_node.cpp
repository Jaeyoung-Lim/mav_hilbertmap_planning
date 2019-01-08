//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "hilbert_mapper/hilbert_mapper.h"

using namespace Eigen;
using namespace std;
//Constructor
hilbertMapper::hilbertMapper(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private) {

    cmdloop_timer_ = nh_.createTimer(ros::Duration(0.01), &hilbertMapper::cmdloopCallback, this); // Define timer for constant loop rate
    statusloop_timer_ = nh_.createTimer(ros::Duration(1), &hilbertMapper::statusloopCallback, this); // Define timer for constant loop rate

    mavposeSub_ = nh_.subscribe("/mavros/local_position/pose", 1, &hilbertMapper::mavposeCallback, this,ros::TransportHints().tcpNoDelay());

}
hilbertMapper::~hilbertMapper() {
  //Destructor
}

void hilbertMapper::cmdloopCallback(const ros::TimerEvent& event) {



    ros::spinOnce();
}

void hilbertMapper::statusloopCallback(const ros::TimerEvent &event) {

}

void hilbertMapper::mavposeCallback(const geometry_msgs::PoseStamped& msg){
    mavPos_(0) = msg.pose.position.x;
    mavPos_(1) = msg.pose.position.y;
    mavPos_(2) = msg.pose.position.z;
    mavAtt_(0) = msg.pose.orientation.w;
    mavAtt_(1) = msg.pose.orientation.x;
    mavAtt_(2) = msg.pose.orientation.y;
    mavAtt_(3) = msg.pose.orientation.z;

}