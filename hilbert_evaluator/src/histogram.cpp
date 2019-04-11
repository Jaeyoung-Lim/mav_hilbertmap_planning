//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "hilbert_evaluator/histogram.h"

Histogram::Histogram(int binsize, double width)
  : binsize_(binsize),
    width_(width) {

  bin_values_.resize(binsize_);
  bin_limits_.resize(binsize_);
  double resolution;

//Initialize bins

  resolution = width_/binsize_;

  for(int i = 0; i < binsize; i++){
    bin_values_[i] = 0;
    bin_limits_[i] = resolution * (i - 0.5 * binsize_);
  }
}

Histogram::~Histogram() {
  //Destructor
}

void Histogram::add(double value){
  for(int i = 0; i < bin_values_.size() ; i++){
    if(value > bin_limits_[i]){
      continue;
    } else{
      bin_values_[i-1]++;
      break;
    }
  }
}

void Histogram::Save(){
  //Save results to a csv file

}

void Histogram::PrintValues(){
  for(int i = 0; i < binsize_; i++){
    std::cout << bin_values_[i] << ", "; 
  }
  std::cout << std::endl;
}