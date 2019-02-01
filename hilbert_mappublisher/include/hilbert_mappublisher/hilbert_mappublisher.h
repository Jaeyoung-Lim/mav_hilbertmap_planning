//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#ifndef HILBERT_MAPPER_H
#define HILBERT_MAPPER_H

#include <ros/ros.h>


class hilbertMapPublisher
{
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Timer cmdloop_timer_;

    void cmdloopCallback(const ros::TimerEvent& event);

public:
    hilbertMapPublisher(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    virtual ~ hilbertMapPublisher();
};

#endif