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
    eta_(0.7),
    width_(1.0),
    resolution_(0.1) {

    int obs_size = 10;
    Eigen::Vector3d mesh_obs;
    Eigen::Vector3d init_mapcenter;

    init_mapcenter << 0.0, 0.0, 0.0;

    generateGridPoints(anchorpoints_, init_mapcenter, width_, width_, width_, resolution_);

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
    ros::Time start_time;

    start_time = ros::Time::now();

    for(int i = 0; i < std::min(max_iterations_, bin_size); i ++){
        for(int j = 0; j < std::min(num_samples_, bin_size); j++){
            idx[j] = std::rand() % bin_size;
        }

        //TODO: Study the effect of A_
            Eigen::VectorXd prev_weights = weights_;
            weights_ = weights_ - eta_ * A_ * getNegativeLikelyhood(idx);
    }

    time_sgd_ = (ros::Time::now() - start_time).toSec();
}

Eigen::VectorXd hilbertmap::getNegativeLikelyhood(int *index){

    Eigen::VectorXd nll;
    Eigen::VectorXd phi_x;
    Eigen::Vector3d query;

    nll = Eigen::VectorXd::Zero(num_features_);
    phi_x = Eigen::VectorXd::Zero(num_features_);

    for(int i = 0; i < sizeof(index)/sizeof(index[0]); i++){
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

void hilbertmap::generateGridPoints(std::vector<Eigen::Vector3d> &gridpoints, Eigen::Vector3d center, double width, double length, double height, int resolution){

    int num_cells[3];

    num_cells[0]= int(width/resolution);
    num_cells[1] = int(length/resolution);
    num_cells[2] = int(height/resolution);

    Eigen::Vector3d mesh_obs;
    //TODO: This is dirty
    for(int i = 0; i < num_cells[0]; i++){
        for(int j = 0; j < num_cells[1]; j++){
            for(int k = 0; k < num_cells[2]; k++){
                mesh_obs << i * width/double(num_cells[0]) - 0.5 * width, j * length/double(num_cells[1])- 0.5 * length, k * height/double(num_cells[2])- 0.5 * height;
                gridpoints.emplace_back(Eigen::Vector3d(mesh_obs(0), mesh_obs(1), mesh_obs(2)));
            }
        }
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

int hilbertmap::getNumFeatures(){
    return anchorpoints_.size();
}

double hilbertmap::getMapWidth(){
    return width_;
}

double hilbertmap::getMapResolution(){
    return resolution_;
}

double hilbertmap::getSgdTime(){
    return time_sgd_;
}

double hilbertmap::getQueryTime(){
    return time_query_;
}

Eigen::Vector3d hilbertmap::getMapCenter(){
    return map_center_;
}

Eigen::Vector3d hilbertmap::getFeature(int idx){

    Eigen::Vector3d feature;

    feature = anchorpoints_[idx];

    return feature;
}

double hilbertmap::getOccupancyProb(Eigen::Vector3d &x_query){

    double probability;
    Eigen::VectorXd phi_x;
    ros::Time start_time;

    start_time = ros::Time::now();

    phi_x = Eigen::VectorXd::Zero(num_features_);
    getkernelVector(x_query, phi_x);
    probability = 1 / ( 1 + exp(weights_.dot(phi_x)));

    time_query_ = (ros::Time::now() - start_time).toSec();

    return probability;
}