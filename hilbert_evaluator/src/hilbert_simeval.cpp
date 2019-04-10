//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "hilbert_evaluator/hilbert_simeval.h"

namespace voxblox {
HSimulationServerImpl::HSimulationServerImpl(const ros::NodeHandle& nh,
                      const ros::NodeHandle& nh_private)
    : SimulationServer(nh, nh_private) {
  
  hilbertmapPub_ = nh_.advertise<sensor_msgs::PointCloud2>("/hilbert_mapper/hilbertmap", 1);
  binPub_ = nh_.advertise<sensor_msgs::PointCloud2>("/hilbert_mapper/binpoints", 1);

  hilbertMap_.reset(new hilbertmap(1000));

  double num_tests = 20;
  test_thresholds_.resize(num_tests);
  tp.resize(num_tests);
  fn.resize(num_tests);
  fp.resize(num_tests);
  tn.resize(num_tests);

  for(size_t i = 0; i < num_tests ; i++){
    test_thresholds_[i] = i * 1 / double(num_tests);
  }
}

HSimulationServerImpl::~HSimulationServerImpl(){};


void HSimulationServerImpl::prepareWorld() {
  world_.addObject(std::unique_ptr<Object>(
      new Sphere(Point(0.0, 0.0, 2.0), 2.0, Color::Red())));

  world_.addObject(std::unique_ptr<Object>(new PlaneObject(
      Point(-2.0, -4.0, 2.0), Point(0, 1, 0), Color::White())));

  world_.addObject(std::unique_ptr<Object>(
      new PlaneObject(Point(4.0, 0.0, 0.0), Point(-1, 0, 0), Color::Pink())));

  world_.addObject(std::unique_ptr<Object>(
      new Cube(Point(-4.0, 4.0, 2.0), Point(4, 4, 4), Color::Green())));

  world_.addGroundLevel(0.03);

  world_.generateSdfFromWorld(truncation_distance_, tsdf_gt_.get());
  world_.generateSdfFromWorld(esdf_max_distance_, esdf_gt_.get());
}

void HSimulationServerImpl::hilbertBenchmark(){
  //ESDF evaluation
  prepareWorld();
  generateSDF();
  evaluate();
  visualize();
  int benchmark = 2;
  switch(benchmark){
    case 1 : //Benchmark tsdf and raw bin source
      ROS_INFO("[Hilbert Benchmark] Starting benchmark for Bin source");

      //Hilbert map evaluation from TSDF as a source
      initializeHilbertMap();
      appendBinfromTSDF();
        /**
      * @todo Check how much loops are valid
      * @body Running `learnHilbertMap()` once is not enough
      */
      learnHilbertMap();
      learnHilbertMap();
      learnHilbertMap();
      learnHilbertMap();
      learnHilbertMap();
      evaluateHilbertMap();

      //Hilbert map evaluation from Raw pointcloud as a source
      initializeHilbertMap();
      appendBinfromRaw(1.0);
      learnHilbertMap();
      learnHilbertMap();
      learnHilbertMap();
      learnHilbertMap();
      learnHilbertMap();
      evaluateHilbertMap();

      //Hilbert map evaluation from Sparse Raw pointcloud as a source
      initializeHilbertMap();
      appendBinfromRaw(0.25);
      learnHilbertMap();
      learnHilbertMap();
      learnHilbertMap();
      learnHilbertMap();
      learnHilbertMap();
      evaluateHilbertMap();

      //Hilbert map evaluation from Sparse Raw pointcloud as a source
      initializeHilbertMap();
      appendBinfromRaw(0.1);
      learnHilbertMap();
      learnHilbertMap();
      learnHilbertMap();
      learnHilbertMap();
      learnHilbertMap();
      evaluateHilbertMap();

      //Hilbert map evaluation from Sparse Raw pointcloud as a source
      initializeHilbertMap();
      appendBinfromRaw(0.05);
      learnHilbertMap();
      learnHilbertMap();
      learnHilbertMap();
      learnHilbertMap();
      learnHilbertMap();
      evaluateHilbertMap();

      break;
    case 2 : //Bench mark resoultion
      ROS_INFO("[Hilbert Benchmark] Starting benchmark for Map Resoultion");
      //Hilbert map evaluation from TSDF as a source
      initializeHilbertMap(1.1);
      appendBinfromTSDF();
      learnHilbertMap();
      learnHilbertMap();
      learnHilbertMap();
      learnHilbertMap();
      learnHilbertMap();
      evaluateHilbertMap();

      initializeHilbertMap(0.55);
      appendBinfromTSDF();
      learnHilbertMap();
      learnHilbertMap();
      learnHilbertMap();
      learnHilbertMap();
      learnHilbertMap();
      evaluateHilbertMap();


      initializeHilbertMap(2.22);
      appendBinfromTSDF();
      learnHilbertMap();
      learnHilbertMap();
      learnHilbertMap();
      learnHilbertMap();
      learnHilbertMap();
      evaluateHilbertMap();

      break;

  }


  ROS_INFO_STREAM("Timings for Raw pointcloud Source: "
                << std::endl
                << voxblox::timing::Timing::Print() << std::endl);

  while(true){
    visualizeHilbertMap();
    ros::Duration(2.0).sleep();
  }

  return;
}

void HSimulationServerImpl::initializeHilbertMap(){
  int num_samples = 100;
  double width = 11.0;
  double height = 11.0;
  double length = 11.0;
  double resolution = 1.1;
  double tsdf_threshold = 0.0;
  Eigen::Vector3d center_pos;
  center_pos << 0.5, 0.5, 4.5;

  hilbertMap_->setMapProperties(num_samples, width, length, height, resolution, tsdf_threshold);
  hilbertMap_->setMapCenter(center_pos);
}

void HSimulationServerImpl::initializeHilbertMap(double resolution){
  ROS_INFO("Create map with resolution %f", resolution);
  int num_samples = 100;
  double width = 11.0;
  double height = 11.0;
  double length = 11.0;
  double tsdf_threshold = 0.0;
  Eigen::Vector3d center_pos;
  center_pos << 0.5, 0.5, 4.5;

  hilbertMap_->setMapProperties(num_samples, width, length, height, resolution, tsdf_threshold);
  hilbertMap_->setMapCenter(center_pos);
}

void HSimulationServerImpl::generateSDF() {
  Pointcloud ptcloud;
  Colors colors;

  Point view_origin(0.0, 0.0, 2.0);
  Point view_direction(0.0, 1.0, 0.0);
  view_direction.normalize();

  // Save raw pointclouds for evaluation
  view_ptcloud_.resize(num_viewpoints_);
  view_origin_.resize(num_viewpoints_);

  pcl::PointCloud<pcl::PointXYZRGB> ptcloud_pcl;

  for (int i = 0; i < num_viewpoints_; ++i) {
    //TODO: Get raw bin from view points
    if (!generatePlausibleViewpoint(min_dist_, &view_origin, &view_direction)) {
      ROS_WARN(
          "Could not generate enough viewpoints. Generated: %d, Needed: %d", i,
          num_viewpoints_);
      break;
    }

    ptcloud.clear();
    colors.clear();

    world_.getPointcloudFromViewpoint(view_origin, view_direction,
                                      depth_camera_resolution_, fov_h_rad_,
                                      max_dist_, &ptcloud, &colors);
    view_origin_[i] = view_origin;
    view_ptcloud_[i] = ptcloud;

    // Get T_G_C from ray origin and ray direction.
    Transformation T_G_C(view_origin,
                         Eigen::Quaternion<FloatingPoint>::FromTwoVectors(
                             Point(0.0, 0.0, 1.0), view_direction));

    // Transform back into camera frame.
    Pointcloud ptcloud_C;
    transformPointcloud(T_G_C.inverse(), ptcloud, &ptcloud_C);

    // Put into the real map.
    tsdf_integrator_->integratePointCloud(T_G_C, ptcloud_C, colors);

    if (generate_occupancy_) {
      occ_integrator_->integratePointCloud(T_G_C, ptcloud_C);
    }

    if (add_robot_pose_) {
      esdf_integrator_->addNewRobotPosition(view_origin);
    }

    const bool clear_updated_flag = true;
    if (incremental_) {
      esdf_integrator_->updateFromTsdfLayer(clear_updated_flag);
    }

    // Convert to a XYZRGB pointcloud.
    if (visualize_) {
      ptcloud_pcl.header.frame_id = world_frame_;
      pcl::PointXYZRGB point;
      point.x = view_origin.x();
      point.y = view_origin.y();
      point.z = view_origin.z();
      ptcloud_pcl.push_back(point);

      view_ptcloud_pub_.publish(ptcloud_pcl);
      ros::spinOnce();
    }
  }

  // Generate ESDF in batch.
  if (!incremental_) {
    if (generate_occupancy_) {
      esdf_occ_integrator_->updateFromOccLayerBatch();
    }

    esdf_integrator_->updateFromTsdfLayerBatch();

    // Other batch options for reference:
    // esdf_integrator_->updateFromTsdfLayerBatchFullEuclidean();
    // esdf_integrator_->updateFromTsdfLayerBatchOccupancy();
  }
}

void HSimulationServerImpl::appendBinfromRaw(double sample_rate){
  ROS_INFO("Append Bin from Raw with sample rate %f", sample_rate);
  hilbertMap_->clearBin();
  for(int i = 0; i < num_viewpoints_; i ++){
    hilbertMap_->appendBinfromRaw(view_ptcloud_[i], view_origin_[i], sample_rate); 

  }
}

void HSimulationServerImpl::appendBinfromTSDF(){
  ROS_INFO("Append Bin from TSDF Map");
  //Drop Messages if they are comming in too  fast
  pcl::PointCloud<pcl::PointXYZI> ptcloud;
  ptcloud2.reset(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cropped_ptcloud(new pcl::PointCloud<pcl::PointXYZI>);
  ptcloud.header.frame_id = "world";

  createDistancePointcloudFromTsdfLayer(*tsdf_gt_, &ptcloud); //TODO: This should not be the groundtruth tsdf map

  *ptcloud2 = ptcloud;
  binsize_ = getMapSize(*ptcloud2);

  Eigen::Vector3d map_center;
  // Eigen::Vector3d map_center;
  float map_width, map_length, map_height;

  // Crop PointCloud around map center
  pcl::CropBox<pcl::PointXYZI> boxfilter;
  map_center = hilbertMap_->getMapCenter();
  map_width = 0.5 * float(hilbertMap_->getMapWidth());
  map_length = 0.5 * float(hilbertMap_->getMapLength());
  map_height = 0.5 * float(hilbertMap_->getMapHeight());
  float minX = float(map_center(0) - map_width);
  float minY = float(map_center(1) - map_length);
  float minZ = float(map_center(2) - map_height);
  float maxX = float(map_center(0) + map_width);
  float maxY = float(map_center(1) + map_length);
  float maxZ = float(map_center(2) + map_height);
  boxfilter.setMin(Eigen::Vector4f(minX, minY, minZ, 0.0));
  boxfilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
  boxfilter.setInputCloud(ptcloud2);
  boxfilter.filter(*cropped_ptcloud);
  hilbertMap_->appendBin(*cropped_ptcloud);    
}

void HSimulationServerImpl::learnHilbertMap(){
  hilbertMap_->updateWeights();
}

int HSimulationServerImpl::getMapSize(pcl::PointCloud<pcl::PointXYZI> &ptcloud){
  return ptcloud.points.size();
}

void HSimulationServerImpl::evaluateHilbertMap(){
  //Decide where to query
  ROS_INFO("Start HilbertMap evaluation");
  Eigen::Vector3d x_query;
  for(size_t j = 0; j < test_thresholds_.size() ; j++){ //Initialized Counters
    tp[j] = 0;
    fp[j] = 0;
    tn[j] = 0;
    fn[j] = 0;
  }

      
  for(int i = 0; i < binsize_; i++) {
    double label_hilbertmap, label_esdfmap;
    double occprob;

    pcl::PointXYZI point;
    // Lets use bin as the ground truth map  (which makes no sense!)

    x_query = getQueryPoint(*ptcloud2, i);

    
    hilbertMap_->getOccProbAtPosition(x_query, &occprob);

    label_esdfmap = getGroundTruthLabel(*ptcloud2, i);

    for(size_t j = 0; j < test_thresholds_.size() ; j++){
      label_hilbertmap = getHilbertLabel(occprob, test_thresholds_[j]);
      if(label_esdfmap > 0.0 && label_hilbertmap > 0.0) tp[j]++;
      else if(label_esdfmap > 0.0 && !(label_hilbertmap > 0.0)) fn[j]++;
      else if(!(label_esdfmap > 0.0) && label_hilbertmap > 0.0) fp[j]++;
      else tn[j]++;
    }
  }
  //Get Statistics
  if(binsize_ > 0){
    double tpr, fpr, precision, recall, f1_score;
    //Count Precision and Recall depending on thresholds
    for(size_t j = 0; j < test_thresholds_.size() ; j++){
        tpr = double(tp[j]) / double(tp[j] + fn[j]);
        fpr = double(fp[j]) / double(fp[j] + tn[j]);
        precision = double(tp[j]) / double(tp[j] + fp[j]);
        recall = tpr;

        f1_score = 2 * recall * precision / (recall + precision);
        //TODO: Accumulate f1 score
        std::cout << test_thresholds_[j] << ", " << fpr << ", " << tpr << ", " << f1_score << ";"<< std::endl;
    }
  }
}

Eigen::Vector3d HSimulationServerImpl::getQueryPoint(pcl::PointCloud<pcl::PointXYZI> &ptcloud, int i){
  Eigen::Vector3d pos;
  pos << ptcloud[i].x, ptcloud[i].y, ptcloud[i].z;
  return pos;
}

double HSimulationServerImpl::getGroundTruthLabel(pcl::PointCloud<pcl::PointXYZI> &ptcloud, int i){
  double distance = ptcloud[i].intensity;
  if(distance > 0.0) return -1.0; //Unoccupied
  else return 1.0; //Occupied
}

double HSimulationServerImpl::getHilbertLabel(double occprob, double threshold){
  if(occprob >= threshold) return 1.0;
  else return -1.0;   
}

void HSimulationServerImpl::visualizeHilbertMap(){
  PublishHilbertMap();
  PublishBin();

}

void HSimulationServerImpl::PublishHilbertMap(){
  //Publish Hilbert Map
  sensor_msgs::PointCloud2 hilbert_map_msg;

  pcl::PointCloud<pcl::PointXYZI> pointCloud;
  Eigen::Vector3d x_query;
  //Encode pointcloud data
  for(int i = 0; i < hilbertMap_->getNumFeatures(); i ++) {
      pcl::PointXYZI point;
      x_query = hilbertMap_->getFeature(i);
      point.x = x_query(0);
      point.y = x_query(1);
      point.z = x_query(2);

      point.intensity = hilbertMap_->getOccupancyProb(x_query);


      pointCloud.points.push_back(point);
  }

  pcl::toROSMsg(pointCloud, hilbert_map_msg);

  hilbert_map_msg.header.stamp = ros::Time::now();
  hilbert_map_msg.header.frame_id = world_frame_;

  hilbertmapPub_.publish(hilbert_map_msg);
}

void HSimulationServerImpl::PublishBin(){
  //Publish Bin of Hilbert Map
  sensor_msgs::PointCloud2 binpoint_msg;
  pcl::PointCloud<pcl::PointXYZI> pointCloud;

  for(int i = 0; i < hilbertMap_->getBinSize(); i ++) {
      pcl::PointXYZI point;
      point = hilbertMap_->getbinPoint(i);
      pointCloud.points.push_back(point);
  }
  pcl::toROSMsg(pointCloud, binpoint_msg);

  binpoint_msg.header.stamp = ros::Time::now();
  binpoint_msg.header.frame_id = world_frame_;
  binPub_.publish(binpoint_msg);
}

}  // namespace voxblox