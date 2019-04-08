//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "planner_benchmark/trajectory_plotter.h"

using namespace std;
//Constructor
TrajectoryPlotter::TrajectoryPlotter(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private) {

}
TrajectoryPlotter::~TrajectoryPlotter() {
  //Destructor
}