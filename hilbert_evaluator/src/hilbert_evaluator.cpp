//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "hilbert_evaluator/hilbert_evaluator.h"

using namespace std;
//Constructor
HilbertEvaluator::HilbertEvaluator(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private),
  esdf_server_(nh, nh_private),
  hilbert_mapper_(nh, nh_private) {

    cmdloop_timer_ = nh_.createTimer(ros::Duration(1.0), &HilbertEvaluator::cmdloopCallback, this); // Define timer for constant loop rate
    statusloop_timer_ = nh_.createTimer(ros::Duration(0.5), &HilbertEvaluator::statusloopCallback, this); // Define timer for constant loop rate

    posestampedPub_ = nh_.advertise<geometry_msgs::PoseStamped>("/hilbert_evaluator/pose", 1);
    tfstampedSub_ = nh_.subscribe("/vicon/firefly_sbx/firefly_sbx", 1, &HilbertEvaluator::tfStampedCallback, this,ros::TransportHints().tcpNoDelay());


}
HilbertEvaluator::~HilbertEvaluator() {
  //Destructor
}

void HilbertEvaluator::cmdloopCallback(const ros::TimerEvent& event) {
  
  //Decide where to query
  Eigen::Vector3d x_query;

  int tp = 0;
  int tn = 0;
  int fp = 0;
  int fn = 0;

  int binsize = hilbert_mapper_.getHilbertMapPtr()->getBinSize();
  for(int i = 0; i < binsize; i ++) {
    double label_hilbertmap, label_esdfmap;
    pcl::PointXYZI point;
    // Lets use bin as the ground truth map  (which makes no sense!)

    point = hilbert_mapper_.getHilbertMapPtr()->getbinPoint(i);
    x_query << point.x, point.y, point.z;

    // Get Label from hilbert maps
    label_esdfmap = point.intensity;

    label_hilbertmap = getHilbertLabel(x_query);

    if(label_esdfmap > 0.0 && label_hilbertmap > 0.0) tp++;
    else if(label_esdfmap > 0.0 && !(label_hilbertmap > 0.0)) fn++;
    else if(!(label_esdfmap > 0.0) && label_hilbertmap > 0.0) fp++;
    else tn++;
    // if(!(label_esdfmap) > 0.0 && !(label_hilbertmap > 0.0)) tn++;
  }
  
  if(binsize > 0){
    double recall = double(tp) / double(tp + fn);
    double precision = double(tp) / double(tp + fp);

    double f1_score = 2 * recall * precision / (recall + precision);
    std::cout << "Recall: " << recall << " Precision: " << precision << " F1 Score: " << f1_score << std::endl;
  }
}

void HilbertEvaluator::statusloopCallback(const ros::TimerEvent &event) {
  //Slower loop to publish status / info related topics
  hilbert_mapper_.setMapCenter(mav_pos_);

}

double HilbertEvaluator::getHilbertLabel(Eigen::Vector3d &position){

  double occprob;

  hilbert_mapper_.getHilbertMapPtr()->getOccProbAtPosition(position, &occprob);

  if(occprob > 0.5)    return 1.0;
  else return -1.0;   
}

double HilbertEvaluator::getEsdfLabel(Eigen::Vector3d &position){

  // Get label from esdf maps
  double mapdistance;
  esdf_server_.getEsdfMapPtr()->getDistanceAtPosition(position, &mapdistance);  

  if(mapdistance > 0.0){
    return -1.0; //Unoccupied
  }
  return 1.0;   //Occupied
}

void HilbertEvaluator::tfStampedCallback(const geometry_msgs::TransformStamped& msg){

  geometry_msgs::PoseStamped pose_msg;

  mav_pos_(0) = msg.transform.translation.x;
  mav_pos_(1) = msg.transform.translation.y;
  mav_pos_(2) = msg.transform.translation.z;
  mav_att_(0) = msg.transform.rotation.w;
  mav_att_(1) = msg.transform.rotation.x;
  mav_att_(2) = msg.transform.rotation.y;
  mav_att_(3) = msg.transform.rotation.z;
  
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.header.frame_id = "world";
  pose_msg.pose.position.x = mav_pos_(0);
  pose_msg.pose.position.y = mav_pos_(1);
  pose_msg.pose.position.z = mav_pos_(2);
  pose_msg.pose.orientation.w = mav_att_(0);
  pose_msg.pose.orientation.x = mav_att_(1);
  pose_msg.pose.orientation.y = mav_att_(2);
  pose_msg.pose.orientation.z = mav_att_(3);

  posestampedPub_.publish(pose_msg);
}