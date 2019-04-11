//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "hilbert_evaluator/histogram.h"

Histogram::Histogram(int binsize){

  bin_values_.resize(binsize);

}

Histogram::~Histogram() {
  //Destructor
}

void Histogram::add(double value){
  for(int i = 0; i < bin_values_.size() ; i++){
    
  }
}
