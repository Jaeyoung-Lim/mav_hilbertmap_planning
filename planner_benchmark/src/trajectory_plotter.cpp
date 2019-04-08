//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "planner_benchmark/trajectory_plotter.h"

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
  return;
}

void TrajectoryPlotter::outputPlots(const std::string& filename) {
  // Construct file path.
  std::string path = ros::package::getPath("planner_benchmark");

  std::string plots_path = path + "/plots/" + filename + ".csv";

  FILE* fp = fopen(plots_path.c_str(), "w+");
  if (fp == NULL) {
    return;
  }


  fclose(fp);
}