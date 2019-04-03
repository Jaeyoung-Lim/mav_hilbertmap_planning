//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "hilbert_evaluator/hilbert_simeval.h"

using namespace std;
//Constructor
HilbertSimEvaluator::HilbertSimEvaluator(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private),
  esdf_server_(nh, nh_private),
  hilbert_mapper_(nh, nh_private) {

    cmdloop_timer_ = nh_.createTimer(ros::Duration(1.0), &HilbertSimEvaluator::cmdloopCallback, this); // Define timer for constant loop rate
    statusloop_timer_ = nh_.createTimer(ros::Duration(0.5), &HilbertSimEvaluator::statusloopCallback, this); // Define timer for constant loop rate

    posestampedPub_ = nh_.advertise<geometry_msgs::PoseStamped>("/hilbert_evaluator/pose", 1);
    tfstampedSub_ = nh_.subscribe("/vicon/firefly_sbx/firefly_sbx", 1, &HilbertSimEvaluator::tfStampedCallback, this,ros::TransportHints().tcpNoDelay());

    double num_tests = 10;
    roc_accumulator_.resize(num_tests);
    test_thresholds_.resize(num_tests);
    tp.resize(num_tests);
    fn.resize(num_tests);
    fp.resize(num_tests);
    tn.resize(num_tests);

    for(size_t i = 0; i < num_tests ; i++){
      test_thresholds_[i] = i * 1 / double(num_tests);
    }
}
HilbertSimEvaluator::~HilbertSimEvaluator() {
  //Destructor
}

void HilbertSimEvaluator::cmdloopCallback(const ros::TimerEvent& event) {
  
  //Decide where to query
  Eigen::Vector3d x_query;
  for(size_t j = 0; j < test_thresholds_.size() ; j++){
    tp[j] = 0;
    fp[j] = 0;
    tn[j] = 0;
    fn[j] = 0;
  }


  int binsize = hilbert_mapper_.getHilbertMapPtr()->getBinSize();
      
  for(int i = 0; i < binsize; i++) {
    double label_hilbertmap, label_esdfmap;
    double occprob;

    pcl::PointXYZI point;
    // Lets use bin as the ground truth map  (which makes no sense!)

    point = hilbert_mapper_.getHilbertMapPtr()->getbinPoint(i);
    x_query << point.x, point.y, point.z;

    // Get Label from hilbert maps
    label_esdfmap = point.intensity;
    
    hilbert_mapper_.getHilbertMapPtr()->getOccProbAtPosition(x_query, &occprob);

    for(size_t j = 0; j < test_thresholds_.size() ; j++){
      label_hilbertmap = getHilbertLabel(occprob, test_thresholds_[j]);
      if(label_esdfmap > 0.0 && label_hilbertmap > 0.0) tp[j]++;
      else if(label_esdfmap > 0.0 && !(label_hilbertmap > 0.0)) fn[j]++;
      else if(!(label_esdfmap > 0.0) && label_hilbertmap > 0.0) fp[j]++;
      else tn[j]++;
    }
  }
  if(binsize > 0){
    //Count Precision and Recall depending on thresholds
    for(size_t j = 0; j < test_thresholds_.size() ; j++){
        double tpr = double(tp[j]) / double(tp[j] + fn[j]);
        double fpr = double(fp[j]) / double(fp[j] + tn[j]);
        double precision = double(tp[j]) / double(tp[j] + fp[j]);
        double recall = tpr;
        roc_accumulator_[j].Add(tpr, fpr);

        double f1_score = 2 * recall * precision / (recall + precision);
        //TODO: Accumulate f1 score
    }
  }
}

void HilbertSimEvaluator::statusloopCallback(const ros::TimerEvent &event) {
  //Slower loop to publish status / info related topics
  hilbert_mapper_.setMapCenter(mav_pos_);
  

  double mean_recall;
  double mean_precision;
  std::cout << "Window Samples: " << roc_accumulator_[0].GetWindowSamples() << std::endl;
  for(size_t i = 0; i < test_thresholds_.size(); i++){
      roc_accumulator_[i].GetMeanRecall(mean_recall);
      roc_accumulator_[i].GetMeanPrecision(mean_precision);
      std::cout << test_thresholds_[i] << ", " << mean_precision << ", " << mean_recall << ";"<< std::endl;
  }
}

double HilbertSimEvaluator::getHilbertLabel(double occprob, double threshold){

  if(occprob >= threshold) return 1.0;
  else return -1.0;   
}

double HilbertSimEvaluator::getEsdfLabel(Eigen::Vector3d &position){

  // Get label from esdf maps
  double mapdistance;
  esdf_server_.getEsdfMapPtr()->getDistanceAtPosition(position, &mapdistance);  

  if(mapdistance > 0.0){
    return -1.0; //Unoccupied
  }
  return 1.0;   //Occupied
}

void HilbertSimEvaluator::tfStampedCallback(const geometry_msgs::TransformStamped& msg){

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