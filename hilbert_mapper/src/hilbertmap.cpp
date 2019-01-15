//
// Created by jalim on 08.01.19.
//

#include "hilbert_mapper/hilbertmap.h"

hilbertmap::hilbertmap(int num_features):
    num_features_(num_features),
    obs_resolution_(1.0){

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

void hilbertmap::appendBin(pcl::PointXYZ point, Eigen::Vector3d position) {

    Eigen::Vector3d bearing_v;
    Eigen::Vector3d occupied_point, unoccupied_point;

    occupied_point << point.data[0], point.data[1], point.data[2];

    bearing_v = occupied_point - position;

    for(int i = 0; i < bearing_v.norm()/obs_resolution_; i++){
        unoccupied_point = double(i) * bearing_v / bearing_v.norm() + position;
        bin_.emplace_back(pcl::PointXYZI(-1.0f));
        bin_.back().x = unoccupied_point(0);
        bin_.back().y = unoccupied_point(1);
        bin_.back().z = unoccupied_point(2);
    }
    bin_.emplace_back(pcl::PointXYZI(-1.0f));
    bin_.back().x = occupied_point(0);
    bin_.back().y = occupied_point(1);
    bin_.back().z = occupied_point(2);
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