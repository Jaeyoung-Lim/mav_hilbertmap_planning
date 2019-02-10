//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#ifndef HILBERT_LOCO_PLANNER_H
#define HILBERT_LOCO_PLANNER_H

#include <ros/ros.h>

#include <stdio.h>
#include <cstdlib>
#include <string>
#include <sstream>
#include <loco_planner/loco.h>
#include <mav_planning_common/color_utils.h>
#include <mav_planning_common/path_visualization.h>
#include <mav_planning_common/physical_constraints.h>
#include <mav_planning_common/utils.h>
#include <mav_trajectory_generation/timing.h>
#include <mav_visualization/helpers.h>
#include <voxblox/core/common.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/utils/planning_utils.h>
#include "hilbert_mapper/hilbert_mapper.h"

using namespace std;

class HilbertLocoPlanner
{
public:
    static constexpr int kN = 10;

    HilbertLocoPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    virtual ~ HilbertLocoPlanner();

    void setHilbertMap();

    bool getTrajectoryTowardGoal(
            const mav_msgs::EigenTrajectoryPoint& start,
            const mav_msgs::EigenTrajectoryPoint& goal,
            mav_trajectory_generation::Trajectory* trajectory);

    bool getTrajectoryTowardGoalFromInitialTrajectory(
            double start_time,
            const mav_trajectory_generation::Trajectory& trajectory_in,
            const mav_msgs::EigenTrajectoryPoint& goal,
            mav_trajectory_generation::Trajectory* trajectory);

    bool getTrajectoryBetweenWaypoints(
            const mav_msgs::EigenTrajectoryPoint& start,
            const mav_msgs::EigenTrajectoryPoint& goal,
            mav_trajectory_generation::Trajectory* trajectory);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // Callbacks to bind to loco.
    double getOccProb(const Eigen::Vector3d& position) const;
    double getOccProbAndGradient(const Eigen::Vector3d& position, Eigen::Vector3d* gradient) const;
    double getOccProbAndGradientVector(const Eigen::VectorXd& position, Eigen::VectorXd* gradient) const;

    // Evaluate what we've got here.
    bool isPathCollisionFree(
            const mav_msgs::EigenTrajectoryPointVector& path) const;
    bool isPathFeasible(const mav_msgs::EigenTrajectoryPointVector& path) const;

    // Settings for physical constriants.
    mav_planning::PhysicalConstraints constraints_;

    // General settings.
    bool verbose_;
    bool visualize_;
    std::string frame_id_;

    // Loco settings.
    int num_segments_;
    int num_random_restarts_;
    double random_restart_magnitude_;
    double planning_horizon_m_;

    // Planner.
    loco_planner::Loco<kN> loco_;

    // Map.
    hilbertMapper hilbert_map_;
//    std::shared_ptr<voxblox::TsdfMap> tsdf_map_;
};

#endif