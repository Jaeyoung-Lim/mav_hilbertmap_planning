//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#ifndef HILBERT_MAPPUBLISHER_H
#define HILBERT_MAPPUBLISHER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>


class hilbertMapPublisher
{
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Timer cmdloop_timer_;
    ros::Publisher mapcenter_pub_;

    Eigen::Vector3d mapcenter_;



    void cmdloopCallback(const ros::TimerEvent& event);
    void pubMapCenter();

public:
    hilbertMapPublisher(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    virtual ~ hilbertMapPublisher();
};

#endif