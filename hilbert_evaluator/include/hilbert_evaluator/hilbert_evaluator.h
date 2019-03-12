//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#ifndef HILBERT_EVALUATOR_H
#define HILBERT_EVALUATOR_H

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include "hilbert_mapper/hilbert_mapper.h"
#include <voxblox/utils/timing.h>
#include <voxblox_ros/esdf_server.h>
#include <Eigen/Dense>

class HilbertEvaluator
{
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Timer cmdloop_timer_, statusloop_timer_;

    void cmdloopCallback(const ros::TimerEvent& event);
    void statusloopCallback(const ros::TimerEvent& event);

public:
    HilbertEvaluator(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    virtual ~ HilbertEvaluator();

    voxblox::EsdfServer esdf_server_;
    
    HilbertMapper hilbert_mapper_;

};

#endif //HILBERT_EVALUATOR_H