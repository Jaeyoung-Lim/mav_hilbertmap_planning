//
// Created by jalim on 08.01.19.
//

#include "hilbert_mapper/hilbertmap.h"

hilbertmap::hilbertmap(int num_features):
    num_anchorpoints_(num_features) {

    //TODO: Initialize number of anchorpoints

    //TODO: Initialize weights
//    anchorpoints_ = Eigen::VectorXd::Zero(num_anchorpoints_);



}
hilbertmap::~hilbertmap() {
    //Destructor
}

void hilbertmap::updateWeights(){
//    anchorpoints _ = gradientDescent

}

Eigen::VectorXd hilbertmap::getkernelVector(Eigen::Vector3d x_query){

    Eigen::VectorXd kernel_vector;

    for(int i = 1; i <= num_anchorpoints_; i++){
        kernel_vector(i) = kernel(x_query, anchorpoints_[i]);
    }

    return kernel_vector;
}

double hilbertmap::kernel(Eigen::Vector3d x, Eigen::Vector3d x_hat){
    double kernel;
    double radius, r, sigma;
    sigma = 1.0;

    r = (x - x_hat).norm();

    kernel = exp(-0.5 * pow(r, 2) / pow(sigma, 2));

    return kernel;
}