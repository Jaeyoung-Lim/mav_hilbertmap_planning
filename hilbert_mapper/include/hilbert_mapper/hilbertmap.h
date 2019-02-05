//
// Created by jalim on 08.01.19.
//

#ifndef HILBERT_MAPPER_HILBERTMAP_H
#define HILBERT_MAPPER_HILBERTMAP_H

#include <Eigen/Dense>
#include <cmath>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class hilbertmap
{
    private:

        int num_features_;
        int max_iterations_;
        int num_samples_;
        double eta_; //Learning Rate
        double width_;
        double resolution_;
        double time_query_; // Time for querying a single time
        double time_sgd_; // Convergence time for stochastic gradient descent
        double sigma_; // Covariance for RBF Kernel
        float tsdf_threshold_;

        Eigen::Vector3d pointcloud;
        Eigen::Vector3d map_center_;
        Eigen::VectorXd weights_;
        Eigen::MatrixXd A_;
        std::vector<Eigen::Vector3d> anchorpoints_;
        std::vector<pcl::PointXYZI> bin_;

        Eigen::VectorXd getNegativeLikelyhood(std::vector<int> &index);

public:
        hilbertmap(int num_feature);
        virtual ~hilbertmap();
        void updateWeights();
        void appendBin(pcl::PointCloud<pcl::PointXYZI> &ptcloud);
        void setMapProperties(int num_samples, double width, double resolution, float tsdf_threshold);
        void setMapCenter(Eigen::Vector3d map_center);
        void getkernelVector(Eigen::Vector3d x_query, Eigen::VectorXd &kernel_vector);
        void generateGridPoints(std::vector<Eigen::Vector3d> &gridpoints, Eigen::Vector3d center, double width, double length, double height, double resolution);
        
        int getBinSize();
        int getNumFeatures();
        double getOccupancyProb(Eigen::Vector3d &x_query);
        double getMapWidth();
        double getMapResolution();
        double getSgdTime();
        double getQueryTime();
        Eigen::Vector3d getMapCenter();
        Eigen::Vector3d getFeature(int idx);
        Eigen::VectorXd getWeights();
        pcl::PointXYZI getbinPoint(int idx);

};


#endif //HILBERT_MAPPER_HILBERTMAP_H