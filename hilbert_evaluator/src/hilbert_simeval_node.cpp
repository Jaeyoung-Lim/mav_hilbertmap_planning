//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "hilbert_evaluator/hilbert_simeval.h"
#include "voxblox_ros/simulation_server.h"

namespace voxblox {
class SimulationServerImpl : public voxblox::SimulationServer {
  private:
    std::shared_ptr<hilbertmap> hilbertMap_;

 public:
  SimulationServerImpl(const ros::NodeHandle& nh,
                       const ros::NodeHandle& nh_private)
      : SimulationServer(nh, nh_private) {
      
      hilbertMap_.reset(new hilbertmap(1000));

      }

  void prepareWorld() {
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

  void hilbertBenchmark(){
    //ESDF evaluation
    prepareWorld();
    generateSDF();
    evaluate();
    visualize();
    //Hilbert map evaluation
    initializeHilbertMap();
    appendBinfromTSDF();
    learnHilbertMap();
    // verify();
    // publish();
  }

  void initializeHilbertMap(){
    int num_samples = 100;
    double width = 5.0;
    double height = 5.0;
    double length = 5.0;
    double resolution = 0.5;
    double tsdf_threshold = 0.0;
    Eigen::Vector3d center_pos;
    center_pos << 0.0, 0.0, 0.0;

    hilbertMap_->setMapProperties(num_samples, width, length, height, resolution, tsdf_threshold);
    hilbertMap_->setMapCenter(center_pos);
  }

  void appendBinfromTSDF(){
    //Drop Messages if they are comming in too  fast
    pcl::PointCloud<pcl::PointXYZI> ptcloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptcloud2(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cropped_ptcloud(new pcl::PointCloud<pcl::PointXYZI>);
    ptcloud.header.frame_id = "world";

    createDistancePointcloudFromTsdfLayer(*tsdf_gt_, &ptcloud);

    *ptcloud2 = ptcloud;

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

  void learnHilbertMap(){
    hilbertMap_->updateWeights();
  }
};

}  // namespace voxblox

int main(int argc, char** argv) {
  ros::init(argc, argv, "voxblox_sim");
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  voxblox::SimulationServerImpl sim_eval(nh, nh_private);

  sim_eval.hilbertBenchmark();

  ROS_INFO("Done.");
  ros::spin();
  return 0;
}