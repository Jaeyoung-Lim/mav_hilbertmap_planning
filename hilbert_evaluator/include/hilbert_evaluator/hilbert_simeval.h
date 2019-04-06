//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#ifndef HILBERT_SIMEVAL_H
#define HILBERT_SIMEVAL_H

#include "voxblox_ros/simulation_server.h"
#include "hilbert_mapper/hilbertmap.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>

namespace voxblox {
class HSimulationServerImpl : public voxblox::SimulationServer {
  private:
    std::shared_ptr<hilbertmap> hilbertMap_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptcloud2;
    std::vector<double> test_thresholds_;
    std::vector<int> tp;
    std::vector<int> fn;
    std::vector<int> fp;
    std::vector<int> tn;
    int binsize_;

 public:
    HSimulationServerImpl(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

    void prepareWorld();
    void hilbertBenchmark();
    void initializeHilbertMap();
    void appendBinfromTSDF();
    void learnHilbertMap();
    void evaluateHilbertMap();

    double getGroundTruthLabel(pcl::PointCloud<pcl::PointXYZI> &ptcloud, int i);
    double getHilbertLabel(double occprob, double threshold);
    int getMapSize(pcl::PointCloud<pcl::PointXYZI> &ptcloud);
    Eigen::Vector3d getQueryPoint(pcl::PointCloud<pcl::PointXYZI> &ptcloud, int i);
};
}  // namespace voxblox
#endif