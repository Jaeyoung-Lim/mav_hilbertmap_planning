//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "hilbert_mapper/hilbert_mapper.h"

//using namespace RAI;
int main(int argc, char** argv) {
  ros::init(argc,argv,"geometric_controller");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  hilbertMapper hilbertmapper(nh, nh_private);
  ros::spin();
  return 0;
}
