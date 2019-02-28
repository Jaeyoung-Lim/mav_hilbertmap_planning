//
// Created by jalim on 08.01.19.
//

#include "hilbert_mapper/hilbertmap.h"

hilbertmap::hilbertmap(int num_features):
    num_features_(num_features),
    num_samples_(100),
    max_iterations_(200),
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
        nll =  nll - phi_x * bin_[j].intensity / ( 1 + exp(bin_[j].intensity * weights_.dot(phi_x))); //TODO: Add Regularizor
    }
    return nll;
}

void hilbertmap::appendBin(pcl::PointCloud<pcl::PointXYZI> &ptcloud) {

    std::srand(std::time(nullptr));
    int num_observations = ptcloud.points.size();

    for(int i = 0; i < std::min(num_observations, num_samples_); i++){
        int idx = std::rand() % num_observations;
        //TODO: Should we handle duplicate points?
        if(ptcloud[idx].intensity < tsdf_threshold_) bin_.emplace_back(pcl::PointXYZI(1.0f));
        else bin_.emplace_back(pcl::PointXYZI(-1.0f));

        bin_.back().x = ptcloud[idx].x;
        bin_.back().y = ptcloud[idx].y;
        bin_.back().z = ptcloud[idx].z;
    }
}

void hilbertmap::setMapProperties(int num_samples, double width, double resolution, float tsdf_threshold){

    num_samples_ = num_samples;
    num_features_ = std::pow(int(width / resolution), 3);
    width_ = width;
    resolution_ = resolution;

    A_ = Eigen::MatrixXd::Identity(num_features_, num_features_);
    // Reinitialize weights
    weights_ = Eigen::VectorXd::Zero(num_features_);

    tsdf_threshold_ = tsdf_threshold;

    // Reinitialize anchor points
    anchorpoints_.clear();
    generateGridPoints(anchorpoints_, map_center_, width_, width_, width_, resolution_);

}

void hilbertmap::setMapCenter(Eigen::Vector3d map_center){

    map_center_ = map_center;
    weights_ = Eigen::VectorXd::Zero(num_features_);
    anchorpoints_.clear();
    generateGridPoints(anchorpoints_, map_center, width_, width_, width_, resolution_);

}

void hilbertmap::getkernelVector(const Eigen::Vector3d& x_query, Eigen::VectorXd &kernel_vector) const{
// TODO: Vecotrized calculation is slower

//    Eigen::VectorXd r;
//    Eigen::MatrixXd anchorpoints = Eigen::MatrixXd::Zero(3, anchorpoints_.size());
//    for(int i; i < anchorpoints_.size(); i++){
//        anchorpoints.col(i) = anchorpoints_[i];
//    }
//    r = (anchorpoints.colwise()-=x_query).colwise().squaredNorm();
//    kernel_vector = (-0.5 * ( (1/sigma_) *r.array() ).pow(2)).exp().matrix();
    double kernel, r;

    for(int i = 0; i < kernel_vector.size(); i++)
        kernel_vector(i) = exp(-0.5 * pow((x_query - anchorpoints_[i]).norm()/sigma_, 2));

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
                mesh_obs << i * width/double(num_cells[0]) - 0.5 * width + center(0), j * length/double(num_cells[1])- 0.5 * length  + center(1), k * height/double(num_cells[2])- 0.5 * height  + center(2);
                gridpoints.emplace_back(Eigen::Vector3d(mesh_obs(0), mesh_obs(1), mesh_obs(2)));
            }
        }
    }

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

pcl::PointXYZI hilbertmap::getbinPoint(int idx){
    return bin_[idx];
}

double hilbertmap::getOccupancyProb(const Eigen::Vector3d &x_query) const {

    double probability;
    Eigen::VectorXd phi_x;
    ros::Time start_time;

//    start_time = ros::Time::now();
    phi_x = Eigen::VectorXd::Zero(num_features_);
    getkernelVector(x_query, phi_x);

    probability = 1 / ( 1 + exp((-1.0) * weights_.dot(phi_x)));

//    time_query_ = (ros::Time::now() - start_time).toSec();
//    printf("QueryTime: %6f\n", time_query_);

    return probability;
}

bool hilbertmap::getOccProbAtPosition(const Eigen::Vector3d& x_query, double* occprob) const {
    *occprob = getOccupancyProb(x_query);
    if(*occprob > 1.0 || *occprob < 0.0) return false; //Sanity Check
    return true;
}

bool hilbertmap::getOccProbAndGradientAtPosition(const Eigen::Vector3d &x_query, double* occprob ,Eigen::Vector3d* gradient) const {
    bool success = false;
    double occupancy_prob;
    Eigen::VectorXd phi_x;
    Eigen::Vector3d occupancy_gradient;
    ros::Time start_time;

    Eigen::MatrixXd delta_x = Eigen::MatrixXd::Zero(3, anchorpoints_.size());
    Eigen::MatrixXd anchorpoints = Eigen::MatrixXd::Zero(3, anchorpoints_.size());
    
    phi_x = Eigen::VectorXd::Zero(num_features_);
    getkernelVector(x_query, phi_x);

    occupancy_prob = 1 / ( 1 + exp(weights_.dot(phi_x)));

    //  Calculate gradient
    //  dk = ( -1/(0.5*radius^2) ) * phi_hat .*delta_x;
    for(int i; i < anchorpoints_.size(); i++){ //Copy Anchorpoints
        anchorpoints.col(i) = anchorpoints_[i];
    }

    delta_x = anchorpoints.colwise() - x_query; //TODO: confirm sign
    occupancy_gradient = (-1/(0.5*pow(sigma_, 2))) * delta_x * phi_x;

    *occprob = occupancy_prob;
    *gradient = occupancy_gradient;

    if(occupancy_prob > 1.0 || occupancy_prob < 0.0) return success; //Sanity Check
    success = true;

    return success;
}