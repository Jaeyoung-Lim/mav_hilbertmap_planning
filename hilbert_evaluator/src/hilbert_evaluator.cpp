//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "hilbert_evaluator/hilbert_evaluator.h"

using namespace std;
//Constructor
HilbertEvaluator::HilbertEvaluator(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private),
  tsdf_server_(nh, nh_private),
  hilbert_mapper_(nh, nh_private),
  roc_benchmark_(false) {

    cmdloop_timer_ = nh_.createTimer(ros::Duration(1.0), &HilbertEvaluator::cmdloopCallback, this); // Define timer for constant loop rate
    statusloop_timer_ = nh_.createTimer(ros::Duration(0.5), &HilbertEvaluator::statusloopCallback, this); // Define timer for constant loop rate
    Eigen::Vector3d cow_and_lady_center;
    cow_and_lady_center << 0.0, 0.0, 1.0;

    int num_samples = 100;
    double width = 6.0;
    double length = 6.0;
    double height = 3.0;
    double resolution = 0.3;
    double tsdf_threshold = 0.0;

    hilbert_mapper_.getHilbertMapPtr()->setMapProperties(num_samples, width, length, height, resolution, tsdf_threshold);
    hilbert_mapper_.getHilbertMapPtr()->setMapCenter(cow_and_lady_center, width, length, height);
}

HilbertEvaluator::~HilbertEvaluator() {
  //Destructor
}

void HilbertEvaluator::cmdloopCallback(const ros::TimerEvent& event) {
  
}

void HilbertEvaluator::statusloopCallback(const ros::TimerEvent &event) {

}