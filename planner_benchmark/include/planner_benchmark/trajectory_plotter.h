//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#ifndef TRAJECTORYPLOTTER_H
#define TRAJECTORYPLOTTER_H

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <Eigen/Dense>

class TrajectoryPlotter
{
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

public:
    TrajectoryPlotter(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    virtual ~ TrajectoryPlotter();

};

#endif //HILBERT_EVALUATOR_H