//  April/2019, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_planning_common/physical_constraints.h>
#include <voxblox/simulation/simulation_world.h>
#include <voxblox_loco_planner/voxblox_loco_planner.h>
#include <voxblox_ros/esdf_server.h>

#include <hilbert_loco_planner/hilbert_loco_planner.h>
#include <hilbert_mapper/hilbert_mapper.h>

namespace mav_planning {

class HilbertPlanningBenchmark {
 public:
  enum LocalPlanningMethod { kStraightLine = 0, kLoco };

  struct LocalBenchmarkResult {
    int trial_number = 0;
    int seed = 0;
    double density = 0.0;
    double robot_radius_m = 0.0;
    double v_max = 0.0;
    double a_max = 0.0;
    LocalPlanningMethod local_planning_method;
    bool planning_success = false;
    bool is_collision_free = false;
    bool is_feasible = false;
    int num_replans = 0;
    double distance_from_goal = 0.0;
    double computation_time_sec = 0.0;
    double total_path_time_sec = 0.0;
    double total_path_length_m = 0.0;
    double straight_line_path_length_m = 0.0;
  };

  struct TrajectoryRecorder {
    int trial_number;
    double pos_x;
    double pox_y;
    double pox_z;
    double vel_x;
    double vel_y;
    double vel_z;
    double acc_x;
    double acc_y;
    double acc_z;
  };


  HilbertPlanningBenchmark(const ros::NodeHandle& nh,
                         const ros::NodeHandle& nh_private);

  // General trajectory benchmark tools: call these in order.
  void generateWorld(double density);
  void runLocalBenchmark(int trial_number);
  void runGlobalBenchmark(int trial_number);

  void outputResults(const std::string& filename);

  // Output trajectory of resulting path
  void outputTrajectory(const std::string& filename);

  // Accessors.
  bool visualize() const { return visualize_; }

 private:
  void setupPlanners();

  void generateCustomWorld(const Eigen::Vector3d& size, double density);
  // Generates a synthetic viewpoint, and adds it to the voxblox map.
  void addViewpointToMap(const mav_msgs::EigenTrajectoryPoint& viewpoint);

  double getMapDistance(const Eigen::Vector3d& position) const;
  double getMapDistanceAndGradient(const Eigen::Vector3d& position,
                                   Eigen::Vector3d* gradient) const;

  // Evaluate what we've got here.
  bool isPathCollisionFree(
      const mav_msgs::EigenTrajectoryPointVector& path) const;
  bool isPathFeasible(const mav_msgs::EigenTrajectoryPointVector& path) const;

  // Visualization helpers.
  void appendViewpointMarker(
      const mav_msgs::EigenTrajectoryPoint& point,
      visualization_msgs::MarkerArray* marker_array) const;

  // Path helpers.
  void setYawFromVelocity(double default_yaw,
                          mav_msgs::EigenTrajectoryPointVector* path);

  void UpdateHilbertMap(Eigen::Vector3f view_origin);
  
  void HilbertMapAppendBin(pcl::PointCloud<pcl::PointXYZI> &ptcloud, Eigen::Vector3d map_center);

  TrajectoryRecorder recordTrajectory(const mav_msgs::EigenTrajectoryPointVector& path, int number);

  /*
  // Functions to actually run the planners.

  void fillInPathResults(const mav_msgs::EigenTrajectoryPointVector& path,
                         GlobalBenchmarkResult* result) const;

  bool runGlobalPlanner(const GlobalPlanningMethod planning_method,
                        const mav_msgs::EigenTrajectoryPoint& start,
                        const mav_msgs::EigenTrajectoryPoint& goal,
                        mav_msgs::EigenTrajectoryPointVector* waypoints);
  bool runPathSmoother(const PathSmoothingMethod smoothing_method,
                       const mav_msgs::EigenTrajectoryPointVector& waypoints,
                       mav_msgs::EigenTrajectoryPointVector* path);

                       */

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // ROS stuff.
  ros::Publisher path_marker_pub_;
  ros::Publisher view_ptcloud_pub_;
  ros::Publisher additional_marker_pub_;

  // Settings for physical constriants.
  PhysicalConstraints constraints_;

  // General settings.
  bool verbose_;
  bool visualize_;
  std::string frame_id_;

  // Planning settings.
  double replan_dt_;
  int max_replans_;

  // Map settings.
  Eigen::Vector3d lower_bound_;
  Eigen::Vector3d upper_bound_;
  Eigen::Vector3f hilbertmap_center_;

  // Camera parameters for both the NBVP (and other) camera simulations and
  // the camera simulation with the voxblox sim world.
  Eigen::Vector2i camera_resolution_;
  double camera_fov_h_rad_;
  double camera_min_dist_;
  double camera_max_dist_;
  // Optionally use a different max distance for the cam model vs. the actual
  // simulation.
  double camera_model_dist_;
  // Cached.
  double voxel_size_;
  double density_;

  // Voxblox Server!
  voxblox::EsdfServer esdf_server_;
  voxblox::SimulationWorld world_;

  // Hilbert Map!
  HilbertMapper hilbert_mapper_;

  // Planners will go here!
  HilbertLocoPlanner loco_planner_;

  // Which methods to use.
  std::vector<LocalPlanningMethod> local_planning_methods_;

  // Results.
  std::vector<LocalBenchmarkResult> results_;

  std::vector<TrajectoryRecorder> trajectory_recorder_;

};

}  // namespace mav_planning
