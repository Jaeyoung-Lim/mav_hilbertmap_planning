//  Jan/2019, ETHZ, Jaeyoung Lim, jalim@ethz.ch

#ifndef HILBERT_MAPPUBLISHER_H
#define HILBERT_MAPPUBLISHER_H

#include <ros/ros.h>
#include <Eigen/Dense>

#include <geometry_msgs/PoseStamped.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/PCLPointCloud2.h>


class hilbertMapPublisher
{
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Timer cmdloop_timer_;
    ros::Publisher mapcenterPub_;
    ros::Publisher pointcloudPub_;

    Eigen::Vector3d mapcenter_;

    int map_width_;
    double map_resolution_;

    void cmdloopCallback(const ros::TimerEvent& event);
    void pubMapCenter();
    void pubPointCloud();

public:
    hilbertMapPublisher(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    virtual ~ hilbertMapPublisher();
};

#endif