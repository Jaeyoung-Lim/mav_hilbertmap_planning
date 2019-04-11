//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#ifndef HILBERT_SIMEVAL_H
#define HILBERT_SIMEVAL_H

#include "voxblox_ros/simulation_server.h"
#include "hilbert_mapper/hilbertmap.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include "hilbert_evaluator/histogram.h"

namespace voxblox {
class HSimulationServerImpl : public voxblox::SimulationServer {
  private:

    ros::Publisher hilbertmapPub_;
    ros::Publisher binPub_;

    std::shared_ptr<hilbertmap> hilbertMap_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr ptcloud2;
    std::vector<Pointcloud> view_ptcloud_;

    std::vector<Point> view_origin_;
    std::vector<double> test_thresholds_;
    std::vector<int> tp;
    std::vector<int> fn;
    std::vector<int> fp;
    std::vector<int> tn;

    int binsize_;

 public:
    HSimulationServerImpl(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    ~HSimulationServerImpl();

    void prepareWorld();
    void generateSDF();
    void hilbertBenchmark();
    void initializeHilbertMap();
    void initializeHilbertMap(double resolution);
    void appendBinfromTSDF();
    void appendBinfromRaw(double sample_rate);
    void learnHilbertMap();
    void evaluateHilbertMap();
    void analyzeHilbertMapErrors(double occupancythreshold);
    void visualizeHilbertMap();
    void PublishHilbertMap();
    void PublishBin();

    double getGroundTruthLabel(pcl::PointCloud<pcl::PointXYZI> &ptcloud, int i);
    double getGroundTruthTruncatedDistance(pcl::PointCloud<pcl::PointXYZI> &ptcloud, int i);
    double getHilbertLabel(double occprob, double threshold);
    int getMapSize(pcl::PointCloud<pcl::PointXYZI> &ptcloud);
    Eigen::Vector3d getQueryPoint(pcl::PointCloud<pcl::PointXYZI> &ptcloud, int i);
};
}  // namespace voxblox
#endif