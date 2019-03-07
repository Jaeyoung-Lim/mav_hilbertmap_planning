//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "hilbert_mappublisher/hilbert_mappublisher.h"

//using namespace RAI;
int main(int argc, char** argv) {
  ros::init(argc,argv,"geometric_controller");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  HilbertMapPublisher hilbertmapper(nh, nh_private);
  ros::spin();
  return 0;
}
