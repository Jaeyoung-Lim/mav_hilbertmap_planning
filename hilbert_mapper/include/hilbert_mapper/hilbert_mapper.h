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
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <std_srvs/SetBool.h>
#include <gazebo_msgs/ModelStates.h>

#define MODE_ROTORTHRUST  1
#define MODE_BODYRATE     2
#define MODE_BODYTORQUE   3

using namespace std;
using namespace Eigen;
class hilbertMapper
{
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

  public:
    hilbertMapper(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    virtual ~ hilbertMapper();
};

#endif