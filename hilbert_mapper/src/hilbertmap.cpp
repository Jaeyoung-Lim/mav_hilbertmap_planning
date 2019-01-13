//
// Created by jalim on 08.01.19.
//

#include "hilbert_mapper/hilbertmap.h"

hilbertmap::hilbertmap(int num_features):
    num_features_(num_features) {

    //TODO: Initialize number of anchorpoints
    for(int i = 0; i < num_features_; i++) {
        anchorpoints_.emplace_back(Eigen::VectorXd::Zero(num_features_));
    }

    weights_ = Eigen::VectorXd::Zero(num_features_);

}
hilbertmap::~hilbertmap() {
    //Destructor
}

void hilbertmap::updateWeights(){
    for(int i = 0; i < max_interations_; i ++){
        //TODO: Sample from bin
        std::vector<Eigen::Vector3d> samples;
        samples = drawObservations();
        // Do gradient descent
        //TODO: Study the effect of A_
        weights_ = weights_ - eta_ * A_ * getNegativeLikelyhood();
    }
}

Eigen::VectorXd hilbertmap::getNegativeLikelyhood(){
    Eigen::VectorXd nll, phi_x;
    nll = Eigen::VectorXd::Zero(num_features_);
    //TODO: Implement negative loglikelyhood
    for(int i = 0; i < num_samples_; i++){
//        phi_x = getkernelVector(x[i]);
//        nll -= y[i] * phi_x / ( 1 + exp(y[i]*weights_.dot(phi_x)));
    }
    return nll;
}

Eigen::VectorXd hilbertmap::getkernelVector(Eigen::Vector3d x_query){

    Eigen::VectorXd phi_hat;

    for(int i = 1; i <= num_features_; i++) phi_hat(i) = kernel(x_query, anchorpoints_[i]);

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

Eigen::VectorXd hilbertmap::getWeights(){
    return weights_;
}