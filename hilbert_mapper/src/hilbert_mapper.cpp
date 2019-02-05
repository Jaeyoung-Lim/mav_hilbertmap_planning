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

    cmdloop_timer_ = nh_.createTimer(ros::Duration(2), &hilbertMapper::cmdloopCallback, this); // Define timer for constant loop rate
    statusloop_timer_ = nh_.createTimer(ros::Duration(2), &hilbertMapper::statusloopCallback, this); // Define timer for constant loop rate

    mapinfoPub_ = nh_.advertise<hilbert_msgs::MapperInfo>("/hilbert_mapper/info", 1);
    hilbertmapPub_ = nh_.advertise<sensor_msgs::PointCloud2>("/hilbert_mapper/hilbertmap", 1);
    anchorPub_ = nh_.advertise<sensor_msgs::PointCloud2>("/hilbert_mapper/anchorpoints", 1);
    binPub_ = nh_.advertise<sensor_msgs::PointCloud2>("/hilbert_mapper/binpoints", 1);
    gridmapPub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/hilbert_mapper/gridmap", 1);


    mavposeSub_ = nh_.subscribe("/hilbert_mapper/map_center/posestamped", 1, &hilbertMapper::mavposeCallback, this,ros::TransportHints().tcpNoDelay());
    mavtransformSub_ = nh_.subscribe("/hilbert_mapper/map_center/mavtf", 1, &hilbertMapper::mavtransformCallback, this,ros::TransportHints().tcpNoDelay());
    poseSub_ = nh_.subscribe("/hilbert_mapper/map_center/pose", 1, &hilbertMapper::poseCallback, this,ros::TransportHints().tcpNoDelay());

    pointcloudSub_ = nh_.subscribe("/hilbert_mapper/tsdf_pointcloud", 1, &hilbertMapper::pointcloudCallback, this,ros::TransportHints().tcpNoDelay());

    int num_samples, num_features;

    nh_.param<int>("/hilbert_mapper/num_parsingsampels", num_samples, 10);
    nh_.param<string>("/hilbert_mapper/frame_id", frame_id_, "world");
    nh_.param<double>("/hilbert_mapper/map/resolution", resolution_, 0.1);
    nh_.param<double>("/hilbert_mapper/map/width", width_, 1.0);
    nh_.param<float>("/hilbert_mapper/map/tsdf_threshold", tsdf_threshold_, 0.3);
    nh_.param<bool>("/hilbert_mapper/publsih_hilbertmap", publish_hilbertmap_, true);
    nh_.param<bool>("/hilbert_mapper/publsih_mapinfo", publish_mapinfo_, true);
    nh_.param<bool>("/hilbert_mapper/publsih_gridmap", publish_gridmap_, true);
    nh_.param<bool>("/hilbert_mapper/publsih_anchorpoints", publish_anchorpoints_, true);
    nh_.param<bool>("/hilbert_mapper/publsih_binpoints", publish_binpoints_, true);

    hilbertMap_.setMapProperties(num_samples, width_, resolution_, tsdf_threshold_);
}
hilbertMapper::~hilbertMapper() {
  //Destructor
}

void hilbertMapper::cmdloopCallback(const ros::TimerEvent& event) {

    hilbertMap_.setMapCenter(mavPos_);
    hilbertMap_.updateWeights();
    ros::spinOnce();
}

void hilbertMapper::statusloopCallback(const ros::TimerEvent &event) {
    //Slower loop to publish status / info related topics

    //Reset Map center
    //Pulbish map status related information
    if(publish_mapinfo_) publishMapInfo();
    if(publish_hilbertmap_) publishMap();
    if(publish_gridmap_) publishgridMap();
    if(publish_anchorpoints_) publishAnchorPoints();
    if(publish_binpoints_) publishBinPoints();

}

void hilbertMapper::mavtransformCallback(const geometry_msgs::TransformStamped& msg){
    mavPos_(0) = msg.transform.translation.x;
    mavPos_(1) = msg.transform.translation.y;
    mavPos_(2) = msg.transform.translation.z;
    mavAtt_(0) = msg.transform.rotation.w;
    mavAtt_(1) = msg.transform.rotation.x;
    mavAtt_(2) = msg.transform.rotation.y;
    mavAtt_(3) = msg.transform.rotation.z;

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

    // Crop PointCloud around map center
    pcl::CropBox<pcl::PointXYZI> boxfilter;
    map_center = hilbertMap_.getMapCenter();
    map_width = float(hilbertMap_.getMapWidth());
    float minX = float(map_center(0) - map_width);
    float minY = float(map_center(1) - map_width);
    float minZ = float(map_center(2) - map_width);
    float maxX = float(map_center(0) + map_width);
    float maxY = float(map_center(1) + map_width);
    float maxZ = float(map_center(2) + map_width);
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

    sensor_msgs::PointCloud2 hilbert_map_msg;
    pcl::PointCloud<pcl::PointXYZI> pointCloud;
    Eigen::Vector3d x_query, origin;

    //Encode pointcloud data
    int width_cells = int(width_ / resolution_);
    origin << 0.5 * width_, 0.5 * width_, 0.5 * width_;

    for(int i = 0; i < width_cells; i ++) {
        for (int j = 0; j < width_cells; j++) {
            for (int k = 0; k < width_cells; k++) {
                pcl::PointXYZI point;
                double occ_prob;
                x_query << i * resolution_, j * resolution_ , k * resolution_;
                x_query = x_query - origin;

                point.x = x_query(0);
                point.y = x_query(1);
                point.z = x_query(2);
                point.intensity = hilbertMap_.getOccupancyProb(x_query);

                pointCloud.points.push_back(point);
            }
        }
    }
    pcl::toROSMsg(pointCloud, hilbert_map_msg);

    hilbert_map_msg.header.stamp = ros::Time::now();
    hilbert_map_msg.header.frame_id = frame_id_;

    hilbertmapPub_.publish(hilbert_map_msg);
}

void hilbertMapper::publishgridMap(){

    nav_msgs::OccupancyGrid grid_map;
    int num_features;
    double map_width, map_height; // Map width in m
    double map_resolution;
    std::vector<Eigen::Vector3d> x_grid;
    Eigen::Vector3d x_query, map_center, gridmap_center;

    map_center = hilbertMap_.getMapCenter();
    map_width = hilbertMap_.getMapWidth(); // [m]
    map_resolution = hilbertMap_.getMapResolution();
    gridmap_center << -0.5 * map_width, -0.5 * map_width, 0.0;

    //Publish hilbertmap at anchorpoints through grid
    grid_map.header.stamp = ros::Time::now();
    grid_map.header.frame_id = frame_id_;
    grid_map.info.height = int(map_width / map_resolution); // [cells]
    grid_map.info.width = int(map_width / map_resolution); // [cells]
    grid_map.info.resolution = map_resolution; // [m/cell]
    grid_map.info.origin.position.x = map_center(0) + gridmap_center(0); //origin is the position of cell(0, 0) in the map
    grid_map.info.origin.position.y = map_center(1) + gridmap_center(1);
    grid_map.info.origin.position.z = map_center(2) + gridmap_center(2);
    grid_map.info.origin.orientation.x = 0.0;
    grid_map.info.origin.orientation.y = 0.0;
    grid_map.info.origin.orientation.z = 0.0;
    grid_map.info.origin.orientation.w = 1.0;

    //Get Occupancy information from hilbertmaps
    for (unsigned int x = 0; x < grid_map.info.width; x++){
        for (unsigned int y = 0; y < grid_map.info.height; y++){
            x_query << x * map_resolution, y * map_resolution, 0.0;
            x_query = x_query - map_center + gridmap_center;
            grid_map.data.push_back(int(hilbertMap_.getOccupancyProb(x_query)* 100.0));
        }
    }
    gridmapPub_.publish(grid_map);
}

void hilbertMapper::publishAnchorPoints() {

    sensor_msgs::PointCloud2 anchorpoint_msg;
    pcl::PointCloud<pcl::PointXYZ> pointCloud;
    Eigen::Vector3d x_feature, origin;

    //Encode pointcloud data
    int width_cells = int(width_ / resolution_);

    for(int i = 0; i < hilbertMap_.getNumFeatures(); i ++) {
        pcl::PointXYZ point;
        x_feature = hilbertMap_.getFeature(i);
        point.x = x_feature(0);
        point.y = x_feature(1);
        point.z = x_feature(2);

        pointCloud.points.push_back(point);
    }
    pcl::toROSMsg(pointCloud, anchorpoint_msg);

    anchorpoint_msg.header.stamp = ros::Time::now();
    anchorpoint_msg.header.frame_id = frame_id_;

    anchorPub_.publish(anchorpoint_msg);
}

void hilbertMapper::publishBinPoints() {

    sensor_msgs::PointCloud2 binpoint_msg;
    pcl::PointCloud<pcl::PointXYZI> pointCloud;

    for(int i = 0; i < hilbertMap_.getBinSize(); i ++) {
        pcl::PointXYZI point;
        point = hilbertMap_.getbinPoint(i);
        pointCloud.points.push_back(point);
    }
    pcl::toROSMsg(pointCloud, binpoint_msg);

    binpoint_msg.header.stamp = ros::Time::now();
    binpoint_msg.header.frame_id = frame_id_;

    binPub_.publish(binpoint_msg);
}