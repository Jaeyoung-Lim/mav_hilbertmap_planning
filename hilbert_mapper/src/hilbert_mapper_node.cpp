//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "hilbert_mapper/hilbert_mapper.h"

using namespace Eigen;
using namespace std;
//Constructor
hilbertMapper::hilbertMapper(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private),
  hilbertMap_(hilbertmap(1000)),
  index_pointcloud(0){

    cmdloop_timer_ = nh_.createTimer(ros::Duration(0.1), &hilbertMapper::cmdloopCallback, this); // Define timer for constant loop rate
    statusloop_timer_ = nh_.createTimer(ros::Duration(2), &hilbertMapper::statusloopCallback, this); // Define timer for constant loop rate

    mapinfoPub_ = nh_.advertise<hilbert_msgs::MapperInfo>("/hilbert_mapper/info", 1);
    hilbertmapPub_ = nh_.advertise<sensor_msgs::PointCloud2>("/hilbert_mapper/hilbertmap", 1);
    gridmapPub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/hilbert_mapper/gridmap", 1);

    mavposeSub_ = nh_.subscribe("/mavros/local_position/pose", 1, &hilbertMapper::mavposeCallback, this,ros::TransportHints().tcpNoDelay());
    poseSub_ = nh_.subscribe("/firefly/odometry_sensor1/pose", 1, &hilbertMapper::poseCallback, this,ros::TransportHints().tcpNoDelay());

    pointcloudSub_ = nh_.subscribe("/voxblox_node/tsdf_pointcloud", 1, &hilbertMapper::pointcloudCallback, this,ros::TransportHints().tcpNoDelay());

    int num_samples, num_features;
    double width;

    nh_.param<int>("/hilbert_mapper/num_parsingsampels", num_samples, 10);
    nh_.param<string>("/hilbert_mapper/frame_id", frame_id_, "map");
    nh_.param<double>("/hilbert_mapper/map/resolution", resolution_, 0.1);
    nh_.param<double>("/hilbert_mapper/map/width", width, 1.0);
//    nh_.param<int>("/hilbert_mapper/num_anchorpoints", num_features, 10);

//    hilbertMap_.setMapProperties(num_samples, num_features);
}
hilbertMapper::~hilbertMapper() {
  //Destructor
}

void hilbertMapper::cmdloopCallback(const ros::TimerEvent& event) {

    // TODO: Update hilbertmap from bin (Stochastic gradient descent)
    hilbertMap_.updateWeights();

    ros::spinOnce();
}

void hilbertMapper::statusloopCallback(const ros::TimerEvent &event) {
    //Slower loop to publish status / info related topics

    //Reset Map center
//    hilbertMap_.setMapCenter(mavPos_);

    //Pulbish map status related information
    publishMapInfo();
//    publishMap();
    publishgridMap();

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

void hilbertMapper::poseCallback(const geometry_msgs::Pose& msg){
    mavPos_(0) = msg.position.x;
    mavPos_(1) = msg.position.y;
    mavPos_(2) = msg.position.z;
    mavAtt_(0) = msg.orientation.w;
    mavAtt_(1) = msg.orientation.x;
    mavAtt_(2) = msg.orientation.y;
    mavAtt_(3) = msg.orientation.z;

}


void hilbertMapper::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptcloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cropped_ptcloud(new pcl::PointCloud<pcl::PointXYZI>);
    Eigen::Vector3d map_center;
    float map_width;

    pcl::fromROSMsg(*msg, *ptcloud); //Convert PointCloud2 to PCL vectors

    // Crop PointCloud
    pcl::CropBox<pcl::PointXYZI> boxfilter;
    //TODO: Set reference for cropping point clouds
    map_center = hilbertMap_.getMapCenter();
    map_width = float(hilbertMap_.getMapWidth());
    float minX = map_center(0) - map_width;
    float minY = map_center(1) - map_width;
    float minZ = map_center(2) - map_width;
    float maxX = map_center(0) + map_width;
    float maxY = map_center(1) + map_width;
    float maxZ = map_center(2) + map_width;
    boxfilter.setMin(Eigen::Vector4f(minX, minY, minZ, -1.0));
    boxfilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
    boxfilter.setInputCloud(ptcloud);
    boxfilter.filter(*cropped_ptcloud);

    hilbertMap_.appendBin(*cropped_ptcloud);
}

void hilbertMapper::publishMapInfo(){
    hilbert_msgs::MapperInfo msg;

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id_;
    msg.binsize = uint16_t(hilbertMap_.getBinSize());
    msg.anchorpoints = uint16_t(hilbertMap_.getNumFeatures());
    msg.time_gradientdescent = float(hilbertMap_.getSgdTime());
    msg.time_query = float(hilbertMap_.getQueryTime());

    mapinfoPub_.publish(msg);
}

void hilbertMapper::publishMap(){

    sensor_msgs::PointCloud2 hilbert_map;

    int num_features;
    Eigen::Vector3d x_query;


    hilbert_map.header.stamp = ros::Time::now();
    hilbert_map.header.frame_id = frame_id_;

    num_features = hilbertMap_.getNumFeatures();

//    for(int i = 0; i < num_features; i++){
//        x_query = hilbertMap_.getFeature(i);
//        //Get Occupancy information from hilbertmaps
//        hilbert_map.data[i] = int(hilbertMap_.getOccupancyProb(x_query));
//    }
//    pcl::toROSMsg();
//
    hilbertmapPub_.publish(hilbert_map);
}

void hilbertMapper::publishgridMap(){

    nav_msgs::OccupancyGrid grid_map;
    int num_features;
    double map_width, map_height;
    double map_resolution;
    std::vector<Eigen::Vector3d> x_grid;
    Eigen::Vector3d x_query, map_center;

    double resolution = 0.1; // [m / cells]

    map_center = hilbertMap_.getMapCenter();
    map_width = hilbertMap_.getMapWidth();
    map_resolution = hilbertMap_.getMapResolution();

    //Publish hilbertmap at anchorpoints through grid
    grid_map.header.stamp = ros::Time::now();
    grid_map.header.frame_id = frame_id_;
    grid_map.info.height = int(map_width / resolution);
    grid_map.info.width = int(map_width / resolution);
    grid_map.info.resolution = map_resolution;
    grid_map.info.origin.position.x = map_center(0);
    grid_map.info.origin.position.y = map_center(1);
    grid_map.info.origin.position.z = map_center(2);
    grid_map.info.origin.orientation.x = 0.0;
    grid_map.info.origin.orientation.y = 0.0;
    grid_map.info.origin.orientation.z = 0.0;
    grid_map.info.origin.orientation.w = 1.0;

    //Get Occupancy information from hilbertmaps
    for (unsigned int x = 0; x < grid_map.info.width; x++){
        for (unsigned int y = 0; y < grid_map.info.height; y++){
            x_query << x*grid_map.info.resolution - 0.5 * map_width, y*grid_map.info.resolution - 0.5 * map_width, 0.0*grid_map.info.resolution;
            grid_map.data.push_back(int(hilbertMap_.getOccupancyProb(x_query) * 100.0));
        }
    }

    gridmapPub_.publish(grid_map);
}