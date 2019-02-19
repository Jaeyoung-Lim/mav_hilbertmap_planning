#include "mav_hilbert_planner/mav_hilbert_planner.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "mav_hilbert_planner");
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  FLAGS_alsologtostderr = true;

  mav_planning::MavHilbertPlanner node(nh, nh_private);
  ROS_INFO("Initialized Mav Local Planner node.");

  ros::spin();
  return 0;
}
