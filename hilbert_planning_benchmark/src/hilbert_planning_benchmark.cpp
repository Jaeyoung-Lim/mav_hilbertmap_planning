//  April/2019, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch


#include <mav_planning_common/color_utils.h>
#include <mav_planning_common/path_visualization.h>
#include <mav_planning_common/utils.h>
#include <mav_trajectory_generation/timing.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_visualization/helpers.h>
#include <voxblox/core/common.h>
#include <voxblox/utils/planning_utils.h>

#include "hilbert_planning_benchmark/hilbert_planning_benchmark.h"

namespace mav_planning {

HilbertPlanningBenchmark::HilbertPlanningBenchmark(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      visualize_(true),
      frame_id_("map"),
      replan_dt_(1.0),
      max_replans_(60),
      lower_bound_(0.0, 0.0, 0.0),
      upper_bound_(15.0, 15.0, 5.0),
      camera_resolution_(320, 240),
      camera_fov_h_rad_(1.5708),  // 90 deg
      camera_min_dist_(0.5),
      camera_max_dist_(10.0),
      camera_model_dist_(5.0),
      loco_planner_(nh_, nh_private_),
      esdf_server_(nh_, nh_private_),
      hilbert_mapper_(nh_, nh_private_) {
  constraints_.setParametersFromRos(nh_private_);

  nh_private_.param("visualize", visualize_, visualize_);
  nh_private_.param("frame_id", frame_id_, frame_id_);

  nh_private_.param("camera_max_dist", camera_max_dist_, camera_max_dist_);
  nh_private_.param("camera_model_dist", camera_model_dist_,
                    camera_model_dist_);
  float hilbertmap_x, hilbertmap_y, hilbertmap_z;
  
  nh_private_.param<float>("hilbertmap/center_x", hilbertmap_x, 0.0);
  nh_private_.param<float>("hilbertmap/center_y", hilbertmap_y, 0.0);
  nh_private_.param<float>("hilbertmap/center_z", hilbertmap_z, 0.0);

  hilbertmap_center_ << hilbertmap_x, hilbertmap_y, hilbertmap_z;

  path_marker_pub_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>("path", 1, true);
  view_ptcloud_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >(
      "view_ptcloud_pub", 1, true);
  additional_marker_pub_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>("path_additions",
                                                             1, true);

  esdf_server_.setClearSphere(true);

  loco_planner_.setHilbertMap(hilbert_mapper_.getHilbertMapPtr());

}

void HilbertPlanningBenchmark::generateWorld(double density, int number) {
  // There's a 2 meter padding on each side of the map that's free.
  const double kWorldXY = 15.0;
  const double kWorldZ = 5.0;

  generateCustomWorld(Eigen::Vector3d(kWorldXY, kWorldXY, kWorldZ), density, number);
}

void HilbertPlanningBenchmark::runLocalBenchmark(int trial_number) {
  constexpr double kPlanningHeight = 1.5;
  constexpr double kMinDistanceToGoal = 0.1;

  srand(trial_number);
  esdf_server_.clear();
  LocalBenchmarkResult result_template;

  result_template.trial_number = trial_number;
  result_template.seed = trial_number;
  result_template.density = density_;
  result_template.robot_radius_m = constraints_.robot_radius;
  result_template.v_max = constraints_.v_max;
  result_template.a_max = constraints_.a_max;

  mav_msgs::EigenTrajectoryPoint start, goal;
  start.position_W =
      Eigen::Vector3d(1.0, upper_bound_.y() / 2.0, kPlanningHeight);
  goal.position_W = upper_bound_ - start.position_W;
  goal.position_W.z() = kPlanningHeight;

  start.setFromYaw(0.0);
  goal.setFromYaw(0.0);
  result_template.straight_line_path_length_m =
      (goal.position_W - start.position_W).norm();

  visualization_msgs::MarkerArray marker_array, additional_markers;
  mav_trajectory_generation::Trajectory trajectory;
  mav_trajectory_generation::Trajectory last_trajectory;
  mav_msgs::EigenTrajectoryPointVector executed_path;

  // Clear the visualization markers, if visualizing.
  if (visualize_) {
    visualization_msgs::Marker marker;
    for (int i = 0; i < max_replans_; ++i) {
      // Clear all existing stuff.
      marker.ns = "loco";
      marker.id = i;
      marker.action = visualization_msgs::Marker::DELETEALL;
      marker_array.markers.push_back(marker);
    }
    marker.ns = "executed_path";
    marker.id = 0;
    marker_array.markers.push_back(marker);
    path_marker_pub_.publish(marker_array);
    marker_array.markers.clear();
    ros::spinOnce();
  }

  // In case we're finding new goal if needed, we have to keep track of which
  // goal we're currently tracking.
  mav_msgs::EigenTrajectoryPoint current_goal = goal;

  double start_time = 0.0;
  double plan_elapsed_time = 0.0;
  double total_path_distance = 0.0;
  int i = 0;
  for (i = 0; i < max_replans_; ++i) {
    if (i > 0 && !trajectory.empty()) {
      start_time = replan_dt_;
    }
    // Generate a viewpoint and add it to the map.
    mav_msgs::EigenTrajectoryPoint viewpoint;
    if (i == 0 || executed_path.empty()) {
      viewpoint = start;
    } else {
      viewpoint = executed_path.back();
    }
    addViewpointToMap(viewpoint); //[Hilbert Benchmark] This is where the new viewpoint is added!
    UpdateHilbertMap(viewpoint.position_W.cast<float>());
    if (visualize_) {
      appendViewpointMarker(viewpoint, &additional_markers);
    }

    // Cache last real trajectory.
    last_trajectory = trajectory;

    // Actually plan the path.
    mav_trajectory_generation::timing::MiniTimer timer;
    bool success = false;

    if (i == 0) {
      success = loco_planner_.getTrajectoryTowardGoal(start, goal, &trajectory);
    } else {
      success = loco_planner_.getTrajectoryTowardGoalFromInitialTrajectory(
          start_time, last_trajectory, goal, &trajectory);
    }
    plan_elapsed_time += timer.stop();

    if (!success) {
      break;
    }

    // Sample the trajectory, set the yaw, and append to the executed path.
    mav_msgs::EigenTrajectoryPointVector path;
    mav_trajectory_generation::sampleWholeTrajectory(
        trajectory, constraints_.sampling_dt, &path);
    setYawFromVelocity(start.getYaw(), &path);

    // Append the next stretch of the trajectory. This will also take care of
    // the end.
    size_t max_index = std::min(
        static_cast<size_t>(std::floor(replan_dt_ / constraints_.sampling_dt)),
        path.size() - 1);
    executed_path.insert(executed_path.end(), path.begin(),
                         path.begin() + max_index);
    
    if (visualize_) {
      marker_array.markers.push_back(createMarkerForPath(
          path, frame_id_,
          percentToRainbowColor(static_cast<double>(i) / max_replans_), "loco",
          0.075));
      marker_array.markers.back().id = i;
      path_marker_pub_.publish(marker_array);
      additional_marker_pub_.publish(additional_markers);
      ros::spinOnce();
      ros::Duration(0.05).sleep();
    }

    if ((executed_path.back().position_W - goal.position_W).norm() <
        kMinDistanceToGoal) {
      break;
    }
  }

  if (visualize_) {
    marker_array.markers.push_back(createMarkerForPath(
        executed_path, frame_id_, mav_visualization::Color::Black(),
        "executed_path", 0.1));
    path_marker_pub_.publish(marker_array);
    additional_marker_pub_.publish(additional_markers);
    ros::spinOnce();
  }
  double path_length = computePathLength(executed_path);
  double distance_from_goal = (start.position_W - goal.position_W).norm();
  if (!executed_path.empty()) {
    distance_from_goal =
        (executed_path.back().position_W - goal.position_W).norm();
  }

  result_template.total_path_length_m = path_length;
  result_template.distance_from_goal = distance_from_goal;
  result_template.planning_success = distance_from_goal < kMinDistanceToGoal;
  result_template.num_replans = i;
  result_template.computation_time_sec = plan_elapsed_time;
  // Rough estimate. ;)
  result_template.total_path_time_sec =
      constraints_.sampling_dt * executed_path.size();
  result_template.is_collision_free = isPathCollisionFree(executed_path);
  result_template.is_feasible = isPathFeasible(executed_path);
  result_template.local_planning_method = kLoco;

  results_.push_back(result_template);
  ROS_INFO(
      "[Local Planning Benchmark] Trial number: %d Success: %d Replans: %d "
      "Final path length: %f Distance from goal: %f",
      trial_number, result_template.planning_success, i, path_length,
      distance_from_goal);
  if(isPathCollisionFree(executed_path)){
    TrajectoryRecorder trajectory_template = recordTrajectory(executed_path, trial_number); 
  }
}

void HilbertPlanningBenchmark::runGlobalBenchmark(int trial_number) {
  constexpr double kPlanningHeight = 1.5;
  constexpr double kMinDistanceToGoal = 0.1;

  srand(trial_number);
  esdf_server_.clear();
  LocalBenchmarkResult result_template;

  result_template.trial_number = trial_number;
  result_template.seed = trial_number;
  result_template.density = density_;
  result_template.robot_radius_m = constraints_.robot_radius;
  result_template.v_max = constraints_.v_max;
  result_template.a_max = constraints_.a_max;

  mav_msgs::EigenTrajectoryPoint start, goal;
  start.position_W =
      Eigen::Vector3d(1.0, upper_bound_.y() / 2.0, kPlanningHeight);
  goal.position_W = upper_bound_ - start.position_W;
  goal.position_W.z() = kPlanningHeight;

  start.setFromYaw(0.0);
  goal.setFromYaw(0.0);
  result_template.straight_line_path_length_m =
      (goal.position_W - start.position_W).norm();

  visualization_msgs::MarkerArray marker_array, additional_markers;
  mav_trajectory_generation::Trajectory trajectory;
  mav_trajectory_generation::Trajectory last_trajectory;
  mav_msgs::EigenTrajectoryPointVector executed_path;

  // Clear the visualization markers, if visualizing.
  if (visualize_) {
    visualization_msgs::Marker marker;
    for (int i = 0; i < max_replans_; ++i) {
      // Clear all existing stuff.
      marker.ns = "loco";
      marker.id = i;
      marker.action = visualization_msgs::Marker::DELETEALL;
      marker_array.markers.push_back(marker);
    }
    marker.ns = "executed_path";
    marker.id = 0;
    marker_array.markers.push_back(marker);
    path_marker_pub_.publish(marker_array);
    marker_array.markers.clear();
    ros::spinOnce();
  }

  // In case we're finding new goal if needed, we have to keep track of which
  // goal we're currently tracking.
  mav_msgs::EigenTrajectoryPoint current_goal = goal;

  double start_time = 0.0;
  double plan_elapsed_time = 0.0;
  double total_path_distance = 0.0;

  int i = 0;
  for (i = 0; i < max_replans_; ++i) {
    if (i > 0 && !trajectory.empty()) {
      start_time = replan_dt_;
    }
    // Generate a viewpoint and add it to the map.
    mav_msgs::EigenTrajectoryPoint viewpoint;
    if (i == 0 || executed_path.empty()) {
      viewpoint = start;
    } else {
      viewpoint = executed_path.back();
    }
    addViewpointToMap(viewpoint); //[Hilbert Benchmark] This is where the new viewpoint is added!
    UpdateHilbertMap(hilbertmap_center_); //Fixed map center for global hilbertmap
    if (visualize_) {
      appendViewpointMarker(viewpoint, &additional_markers);
    }

    // Cache last real trajectory.
    last_trajectory = trajectory;

    // Actually plan the path.
    mav_trajectory_generation::timing::MiniTimer timer;
    bool success = false;

    if (i == 0) {
      success = loco_planner_.getTrajectoryTowardGoal(start, goal, &trajectory);
    } else {
      success = loco_planner_.getTrajectoryTowardGoalFromInitialTrajectory(
          start_time, last_trajectory, goal, &trajectory);
    }
    plan_elapsed_time += timer.stop();

    if (!success) {
      break;
    }

    // Sample the trajectory, set the yaw, and append to the executed path.
    mav_msgs::EigenTrajectoryPointVector path;
    mav_trajectory_generation::sampleWholeTrajectory(
        trajectory, constraints_.sampling_dt, &path);
    setYawFromVelocity(start.getYaw(), &path);

    // Append the next stretch of the trajectory. This will also take care of
    // the end.
    size_t max_index = std::min(
        static_cast<size_t>(std::floor(replan_dt_ / constraints_.sampling_dt)),
        path.size() - 1);
    executed_path.insert(executed_path.end(), path.begin(),
                         path.begin() + max_index);

    if (visualize_) {
      marker_array.markers.push_back(createMarkerForPath(
          path, frame_id_,
          percentToRainbowColor(static_cast<double>(i) / max_replans_), "loco",
          0.075));
      marker_array.markers.back().id = i;
      path_marker_pub_.publish(marker_array);
      additional_marker_pub_.publish(additional_markers);
      ros::spinOnce();
      ros::Duration(0.05).sleep();
    }

    if ((executed_path.back().position_W - goal.position_W).norm() <
        kMinDistanceToGoal) {
      break;
    }
  }

  if (visualize_) {
    marker_array.markers.push_back(createMarkerForPath(
        executed_path, frame_id_, mav_visualization::Color::Black(),
        "executed_path", 0.1));
    path_marker_pub_.publish(marker_array);
    additional_marker_pub_.publish(additional_markers);
    ros::spinOnce();
  }
  double path_length = computePathLength(executed_path);
  double distance_from_goal = (start.position_W - goal.position_W).norm();
  if (!executed_path.empty()) {
    distance_from_goal =
        (executed_path.back().position_W - goal.position_W).norm();
  }

  result_template.total_path_length_m = path_length;
  result_template.distance_from_goal = distance_from_goal;
  result_template.planning_success = distance_from_goal < kMinDistanceToGoal;
  result_template.num_replans = i;
  result_template.computation_time_sec = plan_elapsed_time;
  // Rough estimate. ;)
  result_template.total_path_time_sec =
      constraints_.sampling_dt * executed_path.size();
  result_template.is_collision_free = isPathCollisionFree(executed_path);
  result_template.is_feasible = isPathFeasible(executed_path);
  result_template.local_planning_method = kLoco;

  results_.push_back(result_template);
  ROS_INFO(
      "[Local Planning Benchmark] Trial number: %d Success: %d Replans: %d "
      "Final path length: %f Distance from goal: %f",
      trial_number, result_template.planning_success, i, path_length,
      distance_from_goal);
}

void HilbertPlanningBenchmark::outputResults(const std::string& filename) {
  FILE* fp = fopen(filename.c_str(), "w+");
  if (fp == NULL) {
    return;
  }
  fprintf(fp,
          "#trial,seed,density,robot_radius,v_max,a_max,local_method,planning_"
          "success,is_collision_free,is_feasible,num_replans,distance_from_"
          "goal,computation_time_sec,total_path_time_sec,total_path_length_m,"
          "straight_line_path_length_m\n");
  for (const LocalBenchmarkResult& result : results_) {
    fprintf(fp, "%d,%d,%f,%f,%f,%f,%d,%d,%d,%d,%d,%f,%f,%f,%f,%f\n",
            result.trial_number, result.seed, result.density,
            result.robot_radius_m, result.v_max, result.a_max,
            result.local_planning_method, result.planning_success,
            result.is_collision_free, result.is_feasible, result.num_replans,
            result.distance_from_goal, result.computation_time_sec,
            result.total_path_time_sec, result.total_path_length_m,
            result.straight_line_path_length_m);
  }
  fclose(fp);
  ROS_INFO_STREAM("[Hilbert Planning Benchmark] Output results to: " << filename);
}

void HilbertPlanningBenchmark::outputTrajectory(const std::string& filename){
  FILE* fp = fopen(filename.c_str(), "w+");
  if (fp == NULL) {
    return;
  }
  fprintf(fp,
        "#trial,pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, acc_x, acc_y, acc_z\n");
  for (const TrajectoryRecorder& trajectory_template : trajectory_recorder_) {
    fprintf(fp, "%d,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
            trajectory_template.trial_number,
            trajectory_template.pos_x, trajectory_template.pos_y, trajectory_template.pos_z,
            trajectory_template.vel_x, trajectory_template.vel_y, trajectory_template.vel_z,
            trajectory_template.acc_x, trajectory_template.acc_y, trajectory_template.acc_z);
  }

  fclose(fp);
  ROS_INFO_STREAM("[Hilbert Planning Benchmark] Output trajectories to: " << filename);
}

void HilbertPlanningBenchmark::outputEnvironmentStructure(const std::string& filename){
  FILE* fp = fopen(filename.c_str(), "w+");
  if (fp == NULL) {
    return;
  }
  fprintf(fp,
        "#trial, pos_x, pos_y, pos_z, radius, height\n");
  for (const EnvironmentTemplate& environment_template : environment_structure_) {
    for(size_t i = 0; i < environment_template.obstacle_position.size(); i ++){
      fprintf(fp, "%d,%f,%f,%f,%f,%f\n",
        environment_template.trial_number,
        environment_template.obstacle_position[i](0), environment_template.obstacle_position[i](1), environment_template.obstacle_position[i](2),
        environment_template.obstacle_radius[i], environment_template.obstacle_height[i]);
    }
  }
  fclose(fp);
  ROS_INFO_STREAM("[Hilbert Planning Benchmark] Output trajectories to: " << filename);
}

void HilbertPlanningBenchmark::generateCustomWorld(const Eigen::Vector3d& size,
                                                 double density, int number) {
  esdf_server_.clear();

  lower_bound_ = Eigen::Vector3d::Zero();
  upper_bound_ = size;

  density_ = density;  // Cache this for result output.
  world_.clear();
  world_.addPlaneBoundaries(0.0, size.x(), 0.0, size.y());
  world_.addGroundLevel(0.0);
  // Sets the display bounds.
  world_.setBounds(
      Eigen::Vector3f(-1.0, -1.0, -1.0),
      size.cast<voxblox::FloatingPoint>() + Eigen::Vector3f(1.0, 1.0, 1.0));

  // Free space around the edges (so we know we don't start in collision).
  Eigen::Vector3d free_space_bounds(4.0, 4.0, 4.0);

  // Some mins and maxes... All objects gotta be on the floor. Just because.
  const double kMinHeight = 2.0;
  const double kMaxHeight = 5.0;
  const double kMinRadius = 0.25;
  const double kMaxRadius = 1.0;

  double usable_area = (size.x() - 2 * free_space_bounds.x()) *
                       (size.y() - 2 * free_space_bounds.y());
  int num_objects = static_cast<int>(std::floor(density * usable_area));

  EnvironmentTemplate customworld_structure;

  for (int i = 0; i < num_objects; ++i) {
    // First select size; pose depends on size in z.
    double height = randMToN(kMinHeight, kMaxHeight);
    double radius = randMToN(kMinRadius, kMaxRadius);
    // First select its pose.
    Eigen::Vector3d position(
        randMToN(free_space_bounds.x(), size.x() - free_space_bounds.x()),
        randMToN(free_space_bounds.y(), size.y() - free_space_bounds.y()),
        height / 2.0);

    world_.addObject(std::unique_ptr<voxblox::Object>(new voxblox::Cylinder(
        position.cast<float>(), radius, height, voxblox::Color::Gray())));
    customworld_structure.trial_number = number;
    customworld_structure.obstacle_position.push_back(position);
    customworld_structure.obstacle_height.push_back(height);
    customworld_structure.obstacle_radius.push_back(radius);
  }

  environment_structure_.push_back(customworld_structure);

  esdf_server_.setSliceLevel(1.5);

  // Cache the TSDF voxel size.
  voxel_size_ = esdf_server_.getTsdfMapPtr()->getTsdfLayerPtr()->voxel_size();
}

// Generates a synthetic viewpoint, and adds it to the voxblox map.
void HilbertPlanningBenchmark::addViewpointToMap(
    const mav_msgs::EigenTrajectoryPoint& viewpoint) {
  // Step 1: get the T_G_C for this viewpoint, and view origin + direction.
  // The yaw direction is now just coming from the trajectory, as the yaw
  // is assigned earlier for the whole trajectory chunk now.
  Eigen::Vector3f view_origin = viewpoint.position_W.cast<float>();
  Eigen::Vector3f view_direction(1.0, 0.0, 0.0);
  view_direction = viewpoint.orientation_W_B.cast<float>() * view_direction;

  // T_G_C is from z-positive since that's how camera coordinates work.
  voxblox::Transformation T_G_C(
      view_origin.cast<float>(),
      Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f(0.0, 0.0, 1.0),
                                         view_direction));
  // Step 2: actually get the pointcloud.
  voxblox::Pointcloud ptcloud, ptcloud_C;
  voxblox::Colors colors;
  world_.getPointcloudFromViewpoint(view_origin, view_direction,
                                    camera_resolution_, camera_fov_h_rad_,
                                    camera_max_dist_, &ptcloud, &colors);
  
  // Step 3: integrate into the map.
  // Transform back into camera frame.
  voxblox::transformPointcloud(T_G_C.inverse(), ptcloud, &ptcloud_C);
  esdf_server_.integratePointcloud(T_G_C, ptcloud_C, colors);

  // Step 4: update mesh and ESDF. NewPoseCallback will mark unknown as
  // occupied and clear space otherwise.
  esdf_server_.newPoseCallback(T_G_C);
  if (visualize_) {
    esdf_server_.updateMesh();
  } else {
    esdf_server_.updateEsdf();
  }

  if (visualize_) {
    esdf_server_.publishAllUpdatedTsdfVoxels();
    esdf_server_.publishSlices();

    pcl::PointCloud<pcl::PointXYZRGB> ptcloud_pcl;
    ptcloud_pcl.header.frame_id = frame_id_;
    for (size_t i = 0; i < ptcloud.size(); ++i) {
      pcl::PointXYZRGB point;
      point.x = ptcloud[i].x();
      point.y = ptcloud[i].y();
      point.z = ptcloud[i].z();
      point.r = colors[i].r;
      point.g = colors[i].g;
      point.b = colors[i].b;
      ptcloud_pcl.push_back(point);
    }

    view_ptcloud_pub_.publish(ptcloud_pcl);
  }
}

void HilbertPlanningBenchmark::UpdateHilbertMap(Eigen::Vector3f view_origin){
  //Generate Hilbertmap from the TSDF Map
  pcl::PointCloud<pcl::PointXYZI> ptcloud1;
  voxblox::createDistancePointcloudFromTsdfLayer(esdf_server_.getTsdfMapPtr()->getTsdfLayer(), &ptcloud1);
  //Crop and append TSDF map to hilbert map bin
  HilbertMapAppendBin(ptcloud1, view_origin.cast<double>());
  hilbert_mapper_.getHilbertMapPtr()->updateWeights();
}

void HilbertPlanningBenchmark::HilbertMapAppendBin(pcl::PointCloud<pcl::PointXYZI> &ptcloud1, Eigen::Vector3d map_center){

  hilbert_mapper_.setMapCenter(map_center);

  pcl::PointCloud<pcl::PointXYZI>::Ptr ptcloud2;
  ptcloud2.reset(new pcl::PointCloud<pcl::PointXYZI>);
  *ptcloud2 = ptcloud1;

  // Crop PointCloud around map center
  pcl::PointCloud<pcl::PointXYZI>::Ptr cropped_ptcloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::CropBox<pcl::PointXYZI> boxfilter;
  float map_width, map_length, map_height;
  // Eigen::Vector3d map_center = view_origin.cast<double>();
  map_width = 0.5 * float(hilbert_mapper_.getHilbertMapPtr()->getMapWidth());
  map_length = 0.5 * float(hilbert_mapper_.getHilbertMapPtr()->getMapLength());
  map_height = 0.5 * float(hilbert_mapper_.getHilbertMapPtr()->getMapHeight());
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
  // Append bin for hilbert map
  hilbert_mapper_.getHilbertMapPtr()->appendBin(*cropped_ptcloud);
}

double HilbertPlanningBenchmark::getMapDistance(
    const Eigen::Vector3d& position) const {
  double distance = 0.0;
  const bool kInterpolate = true;
  if (!esdf_server_.getEsdfMapPtr()->getDistanceAtPosition(
          position, kInterpolate, &distance)) {
    return 0.0;
  }
  return distance;
}

double HilbertPlanningBenchmark::getMapDistanceAndGradient(
    const Eigen::Vector3d& position, Eigen::Vector3d* gradient) const {
  double distance = 0.0;
  const bool kInterpolate = true;
  if (!esdf_server_.getEsdfMapPtr()->getDistanceAndGradientAtPosition(
          position, kInterpolate, &distance, gradient)) {
    return 0.0;
  }
  return distance;
}

// Evaluate what we've got here.
bool HilbertPlanningBenchmark::isPathCollisionFree(
    const mav_msgs::EigenTrajectoryPointVector& path) const {
  for (const mav_msgs::EigenTrajectoryPoint& point : path) {
    if (getMapDistance(point.position_W) < constraints_.robot_radius) {
      return false;
    }
  }
  return true;
}

bool HilbertPlanningBenchmark::isPathFeasible(
    const mav_msgs::EigenTrajectoryPointVector& path) const {
  // This is easier to check in the trajectory but then we are limited in how
  // we do the smoothing.
  for (const mav_msgs::EigenTrajectoryPoint& point : path) {
    if (point.acceleration_W.norm() > constraints_.a_max + 1e-2) {
      return false;
    }
    if (point.velocity_W.norm() > constraints_.v_max + 1e-2) {
      return false;
    }
  }
  return true;
}

void HilbertPlanningBenchmark::appendViewpointMarker(
    const mav_msgs::EigenTrajectoryPoint& point,
    visualization_msgs::MarkerArray* marker_array) const {
  // First draw axes for these thing....
  visualization_msgs::Marker marker;
  constexpr double kAxesLength = 0.5;
  constexpr double kAxesWidth = 0.05;
  mav_visualization::drawAxes(point.position_W, point.orientation_W_B,
                              kAxesLength, kAxesWidth, &marker);
  marker.header.frame_id = frame_id_;
  marker.ns = "viewpoint_axes";

  marker_array->markers.push_back(marker);
}

void HilbertPlanningBenchmark::setYawFromVelocity(
    double default_yaw, mav_msgs::EigenTrajectoryPointVector* path) {
  for (size_t i = 0; i < path->size(); ++i) {
    // Non-const ref that gets modified below.
    mav_msgs::EigenTrajectoryPoint& point = (*path)[i];

    if (point.velocity_W.norm() > 1e-6) {
      double yaw = atan2(point.velocity_W.y(), point.velocity_W.x());
      point.setFromYaw(yaw);
    } else {
      point.setFromYaw(default_yaw);
    }
  }
}

HilbertPlanningBenchmark::TrajectoryRecorder HilbertPlanningBenchmark::recordTrajectory(
  const mav_msgs::EigenTrajectoryPointVector& path, int number){
  // This is easier to check in the trajectory but then we are limited in how
  // we do the smoothing.
  TrajectoryRecorder record;
  for (const mav_msgs::EigenTrajectoryPoint& point : path) {
    //TODO: Implement trajectory within the actual path window
    record.trial_number = number;
    record.pos_x = point.position_W(0);
    record.pos_y = point.position_W(1);
    record.pos_z = point.position_W(2);
    record.vel_x = point.velocity_W(0);
    record.vel_y = point.velocity_W(1);
    record.vel_z = point.velocity_W(2);

    record.acc_x = point.acceleration_W(0);
    record.acc_y = point.acceleration_W(1);
    record.acc_z = point.acceleration_W(2);

    trajectory_recorder_.push_back(record);
  }
  return record;
}

}  // namespace mav_planning