//  Jan/2019, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "mav_hilbert_navigator/mav_hilbert_navigator.h"

//using namespace RAI;
int main(int argc, char** argv) {
  ros::init(argc,argv,"geometric_controller");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  MavHilbertNavigator hilbertPlanner(nh, nh_private);
  ros::spin();
  return 0;
}
