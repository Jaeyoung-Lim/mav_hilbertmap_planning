//
// Created by jalim on 08.01.19.
//

#include "hilbert_mapper/hilbertmap.h"

hilbertmap::hilbertmap(int num_features):
    num_anchorpoints_(num_features) {

    //TODO: Initialize number of anchorpoints
    for(int i = 0; i < num_anchorpoints_; i++) {
        anchorpoints_.emplace_back(Eigen::VectorXd::Zero(num_anchorpoints_));
    }

    weights_ = Eigen::VectorXd::Zero(num_anchorpoints_);

}
hilbertmap::~hilbertmap() {
    //Destructor
}

void hilbertmap::updateWeights(){
    for(int i = 0; i < max_interations_; i ++){
//        pullfromBin
//        getkernelVector()
    }

}

Eigen::VectorXd hilbertmap::getkernelVector(Eigen::Vector3d x_query){

    Eigen::VectorXd phi_hat;

    for(int i = 1; i <= num_anchorpoints_; i++) phi_hat(i) = kernel(x_query, anchorpoints_[i]);

    return phi_hat;
}

double hilbertmap::kernel(Eigen::Vector3d x, Eigen::Vector3d x_hat){
    double kernel;
    double radius, r, sigma;

    sigma = 1.0;
    r = (x - x_hat).norm();
    kernel = exp(-0.5 * pow(r, 2) / pow(sigma, 2));

    return kernel;
}