//  Jan/2019, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "hilbert_loco_planner/hilbert_loco_planner.h"

using namespace std;
//Constructor
HilbertLocoPlanner::HilbertLocoPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private)
{
}
HilbertLocoPlanner::~HilbertLocoPlanner() {
  //Destructor
}