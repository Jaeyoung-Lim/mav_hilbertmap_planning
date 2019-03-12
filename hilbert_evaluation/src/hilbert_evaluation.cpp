//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "hilbert_evaluation/hilbert_evaluation.h"

using namespace std;
//Constructor
HilbertEvaluation::HilbertEvaluation(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private),
  hilbert_mapper_(nh, nh_private) {

    cmdloop_timer_ = nh_.createTimer(ros::Duration(0.1), &HilbertEvaluation::cmdloopCallback, this); // Define timer for constant loop rate
    statusloop_timer_ = nh_.createTimer(ros::Duration(0.5), &HilbertEvaluation::statusloopCallback, this); // Define timer for constant loop rate

}
HilbertEvaluation::~HilbertEvaluation() {
  //Destructor
}

void HilbertEvaluation::cmdloopCallback(const ros::TimerEvent& event) {

}

void HilbertEvaluation::statusloopCallback(const ros::TimerEvent &event) {
    //Slower loop to publish status / info related topics
}