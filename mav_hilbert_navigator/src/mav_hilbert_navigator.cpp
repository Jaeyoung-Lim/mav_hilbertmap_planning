//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "mav_hilbert_navigator/mav_hilbert_navigator.h"

using namespace std;
//Constructor
MavHilbertNavigator::MavHilbertNavigator(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private)
{
}
MavHilbertNavigator::~MavHilbertNavigator() {
  //Destructor
}