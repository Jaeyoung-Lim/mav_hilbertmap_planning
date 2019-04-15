//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "hilbert_planning_benchmark/trajectory_plotter.h"

using namespace std;
//Constructor
TrajectoryPlotter::TrajectoryPlotter(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private) {

  trajectory_sub_ = nh_private_.subscribe("local_path", 1, &TrajectoryPlotter::PathCallback, this);

}
TrajectoryPlotter::~TrajectoryPlotter() {
  //Destructor
}

void TrajectoryPlotter::PathCallback(const visualization_msgs::MarkerArray& msg) {

  if (msg.markers.empty()) {
    return;
  }
  const visualization_msgs::Marker& marker = msg.markers[0];

  if (marker.points.empty()) {
    return;
  }
  geometry_msgs::Point point = marker.points[0];
  Eigen::Vector3d eigen_point;
  tf::pointMsgToEigen(point, eigen_point);

  double total_distance = 0.0;

  for (size_t i = 1; i < marker.points.size(); ++i) {
    Eigen::Vector3d new_point;
    point = marker.points[i];
    tf::pointMsgToEigen(point, new_point);

    total_distance += (new_point - eigen_point).norm();
    eigen_point = new_point;
  }

  ROS_INFO("Current path length: %f meters", total_distance);
}

void TrajectoryPlotter::outputPlots(const std::string& filename) {
  // Construct file path.
  std::string path = ros::package::getPath("planner_benchmark");
  std::string plots_path = path + "/plots/" + filename + ".csv";

  FILE* fp = fopen(plots_path.c_str(), "w+");
  if (fp == NULL) {
    return;
  }

  fprintf(fp, "#pos_x, pos_y, pos_z\n");

  // for (const BenchmarkResults& result : results_) {
  //   fprintf(fp, "%d,%d,%d,%d,%f,%f,%f\n", result.trial_number, result.type,
  //           result.map_id, result.is_feasible, result.nom_path_length,
  //           result.sol_path_length, result.time_sec);
  // }

  fclose(fp);
}