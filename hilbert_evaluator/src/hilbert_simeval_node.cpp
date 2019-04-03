//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "hilbert_evaluator/hilbert_simeval.h"

//using namespace RAI;
int main(int argc, char** argv) {
  ros::init(argc,argv,"hilbert_simeval");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  HilbertSimEvaluator Hilbertsimevaluator(nh, nh_private);
  ros::spin();
  return 0;
}
