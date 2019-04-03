//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "hilbert_evaluator/hilbert_simeval.h"

//using namespace RAI;
int main(int argc, char** argv) {
  ros::init(argc,argv,"hilbert_simeval");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  HilbertSimEvaluator sim_evaluator(nh, nh_private);

  sim_evaluator.run();

  ROS_INFO("Done.");
  ros::spin();
  return 0;
}
