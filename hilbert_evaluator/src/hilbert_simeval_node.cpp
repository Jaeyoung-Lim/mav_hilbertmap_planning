//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch
#include "hilbert_evaluator/hilbert_simeval.h"


int main(int argc, char** argv) {
  ros::init(argc, argv, "voxblox_sim");
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  voxblox::HSimulationServerImpl sim_eval(nh, nh_private);

  sim_eval.hilbertBenchmark();

  ROS_INFO("Done.");
  ros::spin();
  return 0;
}