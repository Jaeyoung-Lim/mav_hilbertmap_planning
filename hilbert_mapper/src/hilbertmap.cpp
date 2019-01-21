//
// Created by jalim on 08.01.19.
//

#include "hilbert_mapper/hilbertmap.h"

hilbertmap::hilbertmap(int num_features):
    num_features_(num_features),
    obs_resolution_(1.0),
    num_samples_(10){

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
//        samples = drawObservations();
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

void hilbertmap::appendBin(pcl::PointCloud<pcl::PointXYZI> &ptcloud) {

    std::srand(std::time(nullptr));
    int num_observations = ptcloud.points.size();

    for(int i = 0; i < std::min(num_observations, num_samples_); i++){
        int idx = std::rand() % num_observations;

        //TODO: Should we handle duplicate points?
        if(ptcloud[idx].intensity < 0.0) bin_.emplace_back(pcl::PointXYZI(-1.0f));
        else bin_.emplace_back(pcl::PointXYZI(1.0f));

        bin_.back().x = ptcloud[idx].x;
        bin_.back().y = ptcloud[idx].y;
        bin_.back().z = ptcloud[idx].z;
    }

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

int hilbertmap::getBinSize() {
    return bin_.size();
}

int hilbertmap::getNumAnchors(){
    return anchorpoints_.size();

}