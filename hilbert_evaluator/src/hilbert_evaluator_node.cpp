//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "hilbert_evaluator/hilbert_evaluator.h"

//using namespace RAI;
int main(int argc, char** argv) {
  ros::init(argc,argv,"hilbert_evaluator");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  HilbertEvaluator Hilbertevaluator(nh, nh_private);
  ros::spin();
  return 0;
}
