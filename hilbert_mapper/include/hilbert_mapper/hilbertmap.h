//
// Created by jalim on 08.01.19.
//

#ifndef HILBERT_MAPPER_HILBERTMAP_H
#define HILBERT_MAPPER_HILBERTMAP_H


class hilbertmap {
    private:
        Eigen::Vector3d pointcloud;
        Eigen::VectorXd weights;
        Eigen::VectorXd anchorpoints;

        double occupancyprob_;
        Eigen::Vector3d gardient_occupancyprob_;

    public:
        hilbertMap();
        virtual ~hilbertMap();
};


#endif //HILBERT_MAPPER_HILBERTMAP_H