//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#ifndef HILBERT_EVALUATOR_H
#define HILBERT_EVALUATOR_H

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include "hilbert_mapper/hilbert_mapper.h"
#include "hilbert_evaluator/roc_accumulator.h"
#include <voxblox/utils/timing.h>
#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox_ros/esdf_server.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Dense>

class HilbertEvaluator
{
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Timer cmdloop_timer_, statusloop_timer_;
    ros::Publisher posestampedPub_;
    ros::Subscriber tfstampedSub_;

    Eigen::Vector3d mav_pos_;
    Eigen::Vector4d mav_att_;

    std::vector<double> test_thresholds_;
    bool roc_benchmark_;

    std::vector<RocAccumulator> roc_accumulator_;
    std::vector<RocAccumulator> f1_accumulator_;
    std::vector<int> tp;
    std::vector<int> fn;
    std::vector<int> fp;
    std::vector<int> tn;

    void cmdloopCallback(const ros::TimerEvent& event);
    void statusloopCallback(const ros::TimerEvent& event);
    double getHilbertLabel(double occprob, double threshold);
    double getEsdfLabel(Eigen::Vector3d &position);
    void tfStampedCallback(const geometry_msgs::TransformStamped& msg);

public:
    HilbertEvaluator(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    virtual ~ HilbertEvaluator();

    voxblox::EsdfServer esdf_server_;
    
    HilbertMapper hilbert_mapper_;

    std::unique_ptr<MeshIntegrator<TsdfVoxel>> mesh_integrator_;


};

#endif //HILBERT_EVALUATOR_H