//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "hilbert_mapper/hilbert_mapper.h"

using namespace Eigen;
using namespace std;
//Constructor
hilbertMapper::hilbertMapper(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private),
  hilbertMap_(hilbertmap(10)),
  index_pointcloud(0){

    cmdloop_timer_ = nh_.createTimer(ros::Duration(0.01), &hilbertMapper::cmdloopCallback, this); // Define timer for constant loop rate
    statusloop_timer_ = nh_.createTimer(ros::Duration(1), &hilbertMapper::statusloopCallback, this); // Define timer for constant loop rate

    mapinfoPub_ = nh_.advertise<hilbert_msgs::MapperInfo>("/hilbert_mapper/info", 1);

    mavposeSub_ = nh_.subscribe("/mavros/local_position/pose", 1, &hilbertMapper::mavposeCallback, this,ros::TransportHints().tcpNoDelay());
    pointcloudSub_ = nh_.subscribe("/voxblox_node/tsdf_pointcloud", 1, &hilbertMapper::pointcloudCallback, this,ros::TransportHints().tcpNoDelay());

    int num_samples, num_features;

    nh_.param<int>("/hilbert_mapper/num_parsingsampels", num_samples, 10);
//    nh_.param<int>("/hilbert_mapper/num_anchorpoints", num_features, 10);

//    hilbertMap_.setMapProperties(num_samples, num_features);
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
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptcloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cropped_ptcloud(new pcl::PointCloud<pcl::PointXYZI>);
    Eigen::Vector3d map_center;

    pcl::fromROSMsg(*msg, *ptcloud); //Convert PointCloud2 to PCL vectors

    // Crop PointCloud
    pcl::CropBox<pcl::PointXYZI> boxfilter;
    //TODO: Set reference for cropping point clouds
    map_center = hilbertMap_.getMapCenter();
    float minX = map_center(0) - 2.0;
    float minY = map_center(1) - 2.0;
    float minZ = map_center(2) - 2.0;
    float maxX = map_center(0) + 2.0;
    float maxY = map_center(1) + 2.0;
    float maxZ = map_center(2) + 2.0;
    boxfilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
    boxfilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
    boxfilter.setInputCloud(ptcloud);
    boxfilter.filter(*cropped_ptcloud);

    hilbertMap_.appendBin(*cropped_ptcloud);
}

void hilbertMapper::publishMapInfo(){
    hilbert_msgs::MapperInfo msg;

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.binsize = uint16_t(hilbertMap_.getBinSize());
    msg.anchorpoints = uint16_t(hilbertMap_.getNumAnchors());

    mapinfoPub_.publish(msg);
}