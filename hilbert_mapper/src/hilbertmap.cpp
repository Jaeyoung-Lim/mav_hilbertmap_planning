//
// Created by jalim on 08.01.19.
//

#include "hilbert_mapper/hilbertmap.h"

hilbertmap::hilbertmap(int num_features):
    num_features_(num_features),
    num_samples_(30),
    max_iterations_(100),
    weights_(Eigen::VectorXd::Zero(num_features)),
    A_(Eigen::MatrixXd::Identity(num_features, num_features)),
    eta_(0.7),
    width_(1.0),
    resolution_(0.1),
    sigma_(0.1) {

    map_center_ = Eigen::Vector3d::Zero();

    generateGridPoints(anchorpoints_, map_center_, width_, width_, width_, resolution_);


}
hilbertmap::~hilbertmap() {
    //Destructor
}

void hilbertmap::updateWeights(){

    std::srand(std::time(nullptr));
    std::vector<int> idx;
    int bin_size;

    idx.resize(num_samples_);
//    std::cout << idx.size() << std::endl;
    bin_size = bin_.size();
    ros::Time start_time;

    start_time = ros::Time::now();

    for(int i = 0; i < max_iterations_; i ++){
        //Draw samples from bin
        if(bin_size <= 0) return;
        for(int j = 0; j < num_samples_; j++)   idx[j] = std::rand() % bin_size;
        //TODO: Study the effect of A_
        Eigen::VectorXd prev_weights = weights_;
        weights_ = weights_ - eta_ * A_ * getNegativeLikelyhood(idx);
    }
    time_sgd_ = (ros::Time::now() - start_time).toSec();
}

Eigen::VectorXd hilbertmap::getNegativeLikelyhood(std::vector<int> &index){

    Eigen::VectorXd nll;
    Eigen::VectorXd phi_x;
    Eigen::Vector3d query;
    double label;

    nll = Eigen::VectorXd::Zero(num_features_);
    phi_x = Eigen::VectorXd::Zero(num_features_);

    for(int i = 0; i < index.size(); i++){
        int j = index[i];
        query << double(bin_[j].x), double(bin_[j].y), double(bin_[j].z);
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
        if(ptcloud[idx].intensity < 0.5) bin_.emplace_back(pcl::PointXYZI(-1.0f));
        else bin_.emplace_back(pcl::PointXYZI(ptcloud[idx].intensity));

        bin_.back().x = ptcloud[idx].x;
        bin_.back().y = ptcloud[idx].y;
        bin_.back().z = ptcloud[idx].z;
    }
}

void hilbertmap::setMapProperties(int num_samples, double width, double resolution){

    num_samples_ = num_samples;
    num_features_ = int(width / resolution);
    A_ = Eigen::MatrixXd::Identity(num_features_, num_features_);
    // Reinitialize weights
    weights_ = Eigen::VectorXd::Zero(num_features_);

}

void hilbertmap::setMapCenter(Eigen::Vector3d map_center){

    map_center_ = map_center;
    weights_ = Eigen::VectorXd::Zero(num_features_);
    anchorpoints_.clear();
    generateGridPoints(anchorpoints_, map_center, width_, width_, width_, resolution_);

}

void hilbertmap::getkernelVector(Eigen::Vector3d x_query, Eigen::VectorXd &kernel_vector){
    for(int i = 0; i < kernel_vector.size(); i++){
        kernel_vector(i) = kernel(x_query, anchorpoints_[i]);
    }
}

void hilbertmap::generateGridPoints(std::vector<Eigen::Vector3d> &gridpoints, Eigen::Vector3d center, double width, double length, double height, double resolution){

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
    double kernel, r;

    r = (x - x_hat).norm();
    kernel = exp(-0.5 * pow(r/sigma_, 2));

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
//    printf("QueryTime: %6f\n", time_query_);

    return probability;
}