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
#include "hilbert_msgs/MapperInfo.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
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
    ros::Subscriber mavposeSub_;
    ros::Subscriber pointcloudSub_;

    Eigen::Vector3d mavPos_;
    Eigen::Vector4d mavAtt_;
    hilbertmap hilbertMap_;

    void cmdloopCallback(const ros::TimerEvent& event);
    void statusloopCallback(const ros::TimerEvent& event);

    void mavposeCallback(const geometry_msgs::PoseStamped& msg);
    void pointcloudCallback(const PointCloud::ConstPtr& msg);
    void publishMapInfo();


public:
    hilbertMapper(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    virtual ~ hilbertMapper();
};

#endif