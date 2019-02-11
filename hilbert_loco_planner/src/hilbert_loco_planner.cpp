//  Jan/2019, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "hilbert_loco_planner/hilbert_loco_planner.h"

using namespace std;
//Constructor
HilbertLocoPlanner::HilbertLocoPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private),
  loco_(3),
  hilbert_map_(nh, nh_private){

}
HilbertLocoPlanner::~HilbertLocoPlanner() {
  //Destructor
}

void HilbertLocoPlanner::setHilbertMap() {
    //TODO: bind this with hilbertmap

    loco_.setDistanceAndGradientFunction(
            std::bind(&HilbertLocoPlanner::getOccProbAndGradientVector, this,
                      std::placeholders::_1, std::placeholders::_2));
    loco_.setMapResolution(hilbert_map_.voxel_size());
}

bool HilbertLocoPlanner::getTrajectoryTowardGoal(const mav_msgs::EigenTrajectoryPoint& start,
                                            const mav_msgs::EigenTrajectoryPoint& goal,
                                            mav_trajectory_generation::Trajectory* trajectory){
    CHECK_NOTNULL(trajectory);
    trajectory->clear();
    mav_msgs::EigenTrajectoryPoint start_point = start;
    mav_msgs::EigenTrajectoryPoint goal_point = goal;

    // Check if we're already at the goal!
    constexpr double kGoalReachedRange = 0.01;
    if ((goal_point.position_W - start_point.position_W).norm() <
        kGoalReachedRange) {
        if (verbose_) {
            ROS_INFO("[Voxblox Loco Planner] Goal already reached!");
        }
        return true;
    }

    Eigen::Vector3d distance_to_waypoint =
            goal_point.position_W - start_point.position_W;
    double planning_distance =
            (goal_point.position_W - start_point.position_W).norm();
    Eigen::Vector3d direction_to_waypoint = distance_to_waypoint.normalized();

    if (planning_distance > planning_horizon_m_) {
        goal_point.position_W =
                start_point.position_W + planning_horizon_m_ * direction_to_waypoint;
        planning_distance = planning_horizon_m_;
    }

    bool success = getTrajectoryBetweenWaypoints(start_point, goal_point, trajectory);

    return success;
}

double HilbertLocoPlanner::getOccProb(const Eigen::Vector3d& position) const {
    double occprob = 0.0;
    if (!hilbert_map_.getOccProbAtPosition(position, occprob)) {
        return 0.0;
    }
    return occprob;
}

double HilbertLocoPlanner::getOccProbAndGradient(const Eigen::Vector3d& position, Eigen::Vector3d* gradient) const {
    double occprob = 0.0;
    if(!hilbert_map_.getOccProbAndGradientAtPosition(position, occprob, gradient)){
        return 0.0;
    }
    return occprob;

}

double HilbertLocoPlanner::getOccProbAndGradientVector(const Eigen::VectorXd& position, Eigen::VectorXd* gradient) const {
    double occprob = 0.0;

    return occprob;

}