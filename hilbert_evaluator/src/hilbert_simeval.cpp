//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "hilbert_evaluator/hilbert_simeval.h"

using namespace std;
//Constructor
HilbertSimEvaluator::HilbertSimEvaluator(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private),
  esdf_server_(nh, nh_private),
  hilbert_mapper_(nh, nh_private) {

    double num_tests = 10;
    received_gt_esdf_ = false;
    received_gt_tsdf_ =  false;

    roc_accumulator_.resize(num_tests);
    test_thresholds_.resize(num_tests);
    tp.resize(num_tests);
    fn.resize(num_tests);
    fp.resize(num_tests);
    tn.resize(num_tests);

    cmdloop_timer_ = nh_.createTimer(ros::Duration(10.0), &HilbertSimEvaluator::cmdloopCallback, this); // Define timer for constant loop rate

    gt_esdfSub_ = nh_.subscribe("/simulation_eval/esdf_gt", 1, &HilbertSimEvaluator::EsdfPtcloudCallback, this,ros::TransportHints().tcpNoDelay());
    gt_tsdfSub_ = nh_.subscribe("/simulation_eval/tsdf_gt", 1, &HilbertSimEvaluator::TsdfPtcloudCallback, this,ros::TransportHints().tcpNoDelay());

    for(size_t i = 0; i < num_tests ; i++){
      test_thresholds_[i] = i * 1 / double(num_tests);
    }
}
HilbertSimEvaluator::~HilbertSimEvaluator() {
  //Destructor
}

void HilbertSimEvaluator::cmdloopCallback(const ros::TimerEvent& event) {  //Decide where to query
  Eigen::Vector3d x_query;
  for(size_t j = 0; j < test_thresholds_.size() ; j++){
    tp[j] = 0;
    fp[j] = 0;
    tn[j] = 0;
    fn[j] = 0;
  }

  while(!gt_tsdfmap_ || !gt_esdfmap_){
    ROS_INFO_THROTTLE(1.0, "In while loop");
    ros::Duration(1.0).sleep();
  }
  int binsize = getMapSize(*gt_tsdfmap_);

  std::cout << "Pointcloud size: " << binsize << std::endl;

  // int binsize = hilbert_mapper_.getHilbertMapPtr()->getBinSize();
      
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

int HilbertSimEvaluator::getMapSize(pcl::PointCloud<pcl::PointXYZI> &ptcloud){
  return ptcloud.points.size();
}
 

void HilbertSimEvaluator::TsdfPtcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
  gt_tsdfmap_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *gt_tsdfmap_); //Convert PointCloud2 to PCL vectors
  std::cout << "point cloud TSDF received" << std::endl;
  received_gt_tsdf_ = true;
}

void HilbertSimEvaluator::EsdfPtcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
  gt_esdfmap_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *gt_esdfmap_); //Convert PointCloud2 to PCL vectors
  std::cout << "point cloud ESDF received" << std::endl;
  received_gt_esdf_ = true;
}