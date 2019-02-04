//  Jan/2019, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#ifndef MAV_HILBERT_NAVIGATOR_H
#define MAV_HILBERT_NAVIGATOR_H

#include <ros/ros.h>

#include <stdio.h>
#include <cstdlib>
#include <string>
#include <sstream>

using namespace std;

class MavHilbertNavigator
{
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;


public:
    MavHilbertNavigator(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    virtual ~ MavHilbertNavigator();
};

#endif