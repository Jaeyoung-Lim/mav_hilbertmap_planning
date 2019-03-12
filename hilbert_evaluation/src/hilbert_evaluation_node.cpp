//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "hilbert_evaluation/hilbert_evaluation.h"

//using namespace RAI;
int main(int argc, char** argv) {
  ros::init(argc,argv,"hilbert_evaluation");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  HilbertEvaluation Hilbertevaluation(nh, nh_private);
  ros::spin();
  return 0;
}
