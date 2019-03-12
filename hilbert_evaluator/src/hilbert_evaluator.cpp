//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "hilbert_evaluator/hilbert_evaluator.h"

using namespace std;
//Constructor
HilbertEvaluator::HilbertEvaluator(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private),
  esdf_server_(nh, nh_private),
  hilbert_mapper_(nh, nh_private) {

    cmdloop_timer_ = nh_.createTimer(ros::Duration(0.1), &HilbertEvaluator::cmdloopCallback, this); // Define timer for constant loop rate
    statusloop_timer_ = nh_.createTimer(ros::Duration(0.5), &HilbertEvaluator::statusloopCallback, this); // Define timer for constant loop rate

}
HilbertEvaluator::~HilbertEvaluator() {
  //Destructor
}

void HilbertEvaluator::cmdloopCallback(const ros::TimerEvent& event) {
  
  //Decide where to query
  Eigen::Vector3d x_query;
  x_query << 0.0, 0.0, 1.0;

  // Get label from groundtruth map


  // Get label from esdf maps
  double mapdistance;
  esdf_server_.getEsdfMapPtr()->getDistanceAtPosition(x_query, &mapdistance);  

  // Get Label from hilbert maps
  double occprob;
  hilbert_mapper_.getHilbertMapPtr()->getOccProbAtPosition(x_query, &occprob);

  // Compare labels and log results
  std::cout << " - Distance: " << mapdistance <<  " / " << occprob << std::endl;

}

void HilbertEvaluator::statusloopCallback(const ros::TimerEvent &event) {
  //Slower loop to publish status / info related topics
}