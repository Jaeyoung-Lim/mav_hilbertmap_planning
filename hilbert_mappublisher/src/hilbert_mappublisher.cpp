//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "hilbert_mappublisher/hilbert_mappublisher.h"

//Constructor
hilbertMapPublisher::hilbertMapPublisher(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private){

    cmdloop_timer_ = nh_.createTimer(ros::Duration(0.01), &hilbertMapPublisher::cmdloopCallback, this); // Define timer for constant loop rate
}

hilbertMapPublisher::~hilbertMapPublisher() {
  //Destructor
}

void hilbertMapPublisher::cmdloopCallback(const ros::TimerEvent& event) {

    ros::spinOnce();
}