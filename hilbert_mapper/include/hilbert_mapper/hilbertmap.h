//
// Created by jalim on 08.01.19.
//

#ifndef HILBERT_MAPPER_HILBERTMAP_H
#define HILBERT_MAPPER_HILBERTMAP_H

#include <Eigen/Dense>
#include <cmath>

class hilbertmap
{
    private:

        int num_features_;
        int max_interations_;
        int num_samples_;
        double eta_; //Learning Rate

        Eigen::Vector3d pointcloud;
        Eigen::VectorXd weights_;
        Eigen::MatrixXd A_;
        std::vector<Eigen::VectorXd> anchorpoints_;
        std::vector<Eigen::VectorXd> bin_;

        double kernel(Eigen::Vector3d x, Eigen::Vector3d x_hat);
        Eigen::VectorXd getNegativeLikelyhood();

public:
        hilbertmap(int num_feature);
        virtual ~hilbertmap();
        void updateWeights();
        Eigen::VectorXd getkernelVector(Eigen::Vector3d x_query);
        Eigen::VectorXd getWeights();
        int getBinSize();
        int getNumAnchors();
};


#endif //HILBERT_MAPPER_HILBERTMAP_H