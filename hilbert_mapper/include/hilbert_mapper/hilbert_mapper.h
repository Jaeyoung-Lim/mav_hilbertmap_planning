//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#ifndef HILBERT_MAPPER_H
#define HILBERT_MAPPER_H

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <tf/transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>

#include <stdio.h>
#include <cstdlib>
#include <string>
#include <sstream>

#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include "hilbert_msgs/MapperInfo.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>  // fromROSMsg
#include <pcl/filters/crop_box.h>
#include <boost/foreach.hpp>

#include "hilbert_mapper/hilbertmap.h"

using namespace std;
using namespace Eigen;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class hilbertMapper
{
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Timer cmdloop_timer_, statusloop_timer_;
    ros::Publisher mapinfoPub_;
    ros::Publisher hilbertmapPub_;
    ros::Publisher gridmapPub_;
    ros::Publisher anchorPub_;
    ros::Publisher binPub_;
    ros::Subscriber mavposeSub_;
    ros::Subscriber mavtransformSub_;
    ros::Subscriber poseSub_;
    ros::Subscriber pointcloudSub_;

    Eigen::Vector3d mavPos_;
    Eigen::Vector4d mavAtt_;
    hilbertmap hilbertMap_;
    int index_pointcloud;
    string frame_id_;
    double resolution_;
    double width_;
    float tsdf_threshold_;
    bool publish_hilbertmap_, publish_mapinfo_, publish_gridmap_, publish_anchorpoints_, publish_binpoints_;

    void cmdloopCallback(const ros::TimerEvent& event);
    void statusloopCallback(const ros::TimerEvent& event);

    void mavposeCallback(const geometry_msgs::PoseStamped& msg);
    void mavtransformCallback(const geometry_msgs::TransformStamped& msg);
    void poseCallback(const geometry_msgs::Pose& msg);
    void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void publishMapInfo();
    void publishMap();
    void publishgridMap();
    void publishAnchorPoints();
    void publishBinPoints();

public:
    hilbertMapper(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    virtual ~ hilbertMapper();
    double voxel_size(){ return 0.5 * resolution_; };
    bool getOccProbAtPosition(const Eigen::Vector3d& x_query, double &occprob) const;
    bool getOccProbAndGradientAtPosition(const Eigen::Vector3d& x_query, double &occprob, Eigen::Vector3d* gradient) const;
};

#endif