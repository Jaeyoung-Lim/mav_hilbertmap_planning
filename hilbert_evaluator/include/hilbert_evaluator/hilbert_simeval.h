//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#ifndef HILBERT_SIMEVAL_H
#define HILBERT_SIMEVAL_H

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include "hilbert_mapper/hilbert_mapper.h"
#include "hilbert_evaluator/roc_accumulator.h"
#include <voxblox/utils/timing.h>
#include <voxblox_ros/esdf_server.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Dense>

class HilbertSimEvaluator
{
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    Eigen::Vector3d mav_pos_;
    Eigen::Vector4d mav_att_;

    std::vector<double> test_thresholds_;

    std::vector<RocAccumulator> roc_accumulator_;
    std::vector<RocAccumulator> f1_accumulator_;
    std::vector<int> tp;
    std::vector<int> fn;
    std::vector<int> fp;
    std::vector<int> tn;

    double getHilbertLabel(double occprob, double threshold);
    double getEsdfLabel(Eigen::Vector3d &position);
    void tfStampedCallback(const geometry_msgs::TransformStamped& msg);

public:
    HilbertSimEvaluator(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    virtual ~ HilbertSimEvaluator();

    void run();

    voxblox::EsdfServer esdf_server_;
    
    HilbertMapper hilbert_mapper_;

};

#endif //HILBERT_EVALUATOR_H