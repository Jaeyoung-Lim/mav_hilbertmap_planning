//  April/2019, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#ifndef TRAJECTORYPLOTTER_H
#define TRAJECTORYPLOTTER_H

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <ros/package.h>
#include <Eigen/Dense>
#include "visualization_msgs/MarkerArray.h"

class TrajectoryPlotter
{
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber trajectory_sub_;

public:
    TrajectoryPlotter(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    virtual ~ TrajectoryPlotter();
    void PathCallback(const visualization_msgs::MarkerArray& msg);
    void outputPlots(const std::string& filename);
};

#endif //TRAJECTORYPLOTTER_H