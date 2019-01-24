//
// Created by jalim on 08.01.19.
//

#include "hilbert_mapper/hilbertmap.h"

hilbertmap::hilbertmap(int num_features):
    num_features_(num_features),
    obs_resolution_(1.0),
    num_samples_(10),
    max_iterations_(100),
    weights_(Eigen::VectorXd::Zero(num_features)),
    A_(Eigen::MatrixXd::Identity(num_features, num_features)),
    eta_(0.7){

    for(int i = 0; i < num_features_; i++) {
        //TODO: Initialize Anchorpoints to grid
        anchorpoints_.emplace_back(Eigen::Vector3d::Zero());
    }
    map_center_ = Eigen::Vector3d::Zero();

}
hilbertmap::~hilbertmap() {
    //Destructor
}

void hilbertmap::updateWeights(){

    std::srand(std::time(nullptr));
    int idx[num_samples_];
    int k;
    int bin_size = bin_.size();

    for(int i = 0; i < std::min(max_iterations_, bin_size); i ++){
        for(int j = 0; j < std::min(num_samples_, bin_size); j++)
            idx[j] = std::rand() % bin_size;
            //TODO: Study the effect of A_
            weights_ = weights_ - eta_ * A_ * getNegativeLikelyhood(idx);
    }
}

Eigen::VectorXd hilbertmap::getNegativeLikelyhood(int *index){

    Eigen::VectorXd nll;
    Eigen::VectorXd phi_x(num_features_);
    Eigen::Vector3d query;
    nll = Eigen::VectorXd::Zero(num_features_);

    for(int i = 0; i < sizeof(index); i++){
        int j = index[i];
        query << bin_[j].x, bin_[j].y, bin_[j].z;
        getkernelVector(query, phi_x);
        nll =  nll - phi_x * bin_[j].intensity / ( 1 + exp(bin_[j].intensity * weights_.dot(phi_x)));
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

void hilbertmap::setMapProperties(int num_samples, int num_features){

    num_samples_ = num_samples;
    num_features_ = num_features;
    // Reinitialize weights
    weights_ = Eigen::VectorXd::Zero(num_features_);

}

void hilbertmap::setMapCenter(Eigen::Vector3d map_center){
    map_center_ = map_center;

}

void hilbertmap::getkernelVector(Eigen::Vector3d x_query, Eigen::VectorXd &kernel_vector){

    for(int i = 0; i < kernel_vector.size(); i++){
        kernel_vector(i) = kernel(x_query, anchorpoints_[i]);
    }
}

double hilbertmap::kernel(Eigen::Vector3d x, Eigen::Vector3d x_hat){
    double kernel;
    double radius, r, sigma;

    sigma = 1.0;
    r = (x - x_hat).norm();
    kernel = exp(-0.5 * pow(r/sigma, 2));

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

Eigen::Vector3d hilbertmap::getMapCenter(){
    return map_center_;
}