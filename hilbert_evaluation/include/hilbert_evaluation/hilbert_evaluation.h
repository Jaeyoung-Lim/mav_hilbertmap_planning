//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#ifndef HILBERT_EVALUATION_H
#define HILBERT_EVALUATION_H

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include "hilbert_evaluation/hilbert_evaluation.h"
#include "hilbert_mapper/hilbert_mapper.h"
#include <voxblox/utils/timing.h>
#include <voxblox_ros/esdf_server.h>

class HilbertEvaluation
{
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Timer cmdloop_timer_, statusloop_timer_;

    void cmdloopCallback(const ros::TimerEvent& event);
    void statusloopCallback(const ros::TimerEvent& event);

public:
    HilbertEvaluation(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    virtual ~ HilbertEvaluation();

    HilbertMapper hilbert_mapper_;

};

#endif //HILBERT_EVALUATION_H