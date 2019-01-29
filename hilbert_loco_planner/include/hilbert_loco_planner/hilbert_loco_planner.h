//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#ifndef HILBERT_LOCO_PLANNER_H
#define HILBERT_LOCO_PLANNER_H

#include <ros/ros.h>

#include <stdio.h>
#include <cstdlib>
#include <string>
#include <sstream>

using namespace std;

class HilbertLocoPlanner
{
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

public:
    HilbertLocoPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    virtual ~ HilbertLocoPlanner();
};

#endif