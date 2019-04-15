//  April/2019, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "hilbert_planning_benchmark/trajectory_plotter.h"

int main(int argc, char** argv) {
  ros::init(argc,argv,"trajectory_plotter");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  TrajectoryPlotter trajectoryplotter(nh, nh_private);
  ros::spin();
  return 0;
}
