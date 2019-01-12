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

        int num_anchorpoints_;

        Eigen::Vector3d pointcloud;
        Eigen::VectorXd weights;
        std::vector<Eigen::VectorXd> anchorpoints_;
        std::vector<Eigen::VectorXd> bin_;

    public:
        hilbertmap(int num_feature);
        virtual ~hilbertmap();
        void updateWeights();
        Eigen::VectorXd getkernelVector(Eigen::Vector3d x_query);
        double kernel(Eigen::Vector3d x, Eigen::Vector3d x_hat);
};


#endif //HILBERT_MAPPER_HILBERTMAP_H