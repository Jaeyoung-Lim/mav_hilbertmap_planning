//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "hilbert_mapper/hilbert_mapper.h"

using namespace Eigen;
using namespace std;
//Constructor
hilbertMapper::hilbertMapper(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private),
  hilbertMap_(hilbertmap(100)) {

    cmdloop_timer_ = nh_.createTimer(ros::Duration(0.01), &hilbertMapper::cmdloopCallback, this); // Define timer for constant loop rate
    statusloop_timer_ = nh_.createTimer(ros::Duration(1), &hilbertMapper::statusloopCallback, this); // Define timer for constant loop rate

    mapinfoPub_ = nh_.advertise<hilbert_msgs::MapperInfo>("/hilbert_mapper/info", 1);

    mavposeSub_ = nh_.subscribe("/mavros/local_position/pose", 1, &hilbertMapper::mavposeCallback, this,ros::TransportHints().tcpNoDelay());
    pointcloudSub_ = nh_.subscribe("/local_pointcloud", 1, &hilbertMapper::pointcloudCallback, this,ros::TransportHints().tcpNoDelay());

}
hilbertMapper::~hilbertMapper() {
  //Destructor
}

void hilbertMapper::cmdloopCallback(const ros::TimerEvent& event) {

    // TODO: Update hilbertmap from bin (Stochastic gradient descent)
    hilbertMap_.updateWeights();

    // TODO: Publish hilbertmaps after it is learned
    ros::spinOnce();
}

void hilbertMapper::statusloopCallback(const ros::TimerEvent &event) {
    //Slower loop to publish status / info related topics
    publishMapInfo();
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

void hilbertMapper::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg, cloud);

  for(int i = 0; i < cloud.points.size(); ++i){
      hilbertMap_.appendBin(cloud.points[i], mavPos_);
  }

  // TODO: Sample pointclouds


  // TODO: Sample pointclouds to inertial frame with pose


  // TODO: Register occupied / unoccupied points to BIN


  // TODO: Save point clouds into BIN


}

void hilbertMapper::publishMapInfo(){
    hilbert_msgs::MapperInfo msg;

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.binsize = uint16_t(hilbertMap_.getBinSize());
    msg.anchorpoints = uint16_t(hilbertMap_.getNumAnchors());

    mapinfoPub_.publish(msg);
}