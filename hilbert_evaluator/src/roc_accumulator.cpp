//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "hilbert_evaluator/roc_accumulator.h"

RocAccumulator::RocAccumulator() :
  totalsamples_(0),
  windowsamples_(0),
  N(50) {

    sample_precision_.resize(N);
    sample_recall_.resize(N);

    for(int i = 0 ; i < N ; i++){
      sample_precision_[i] = 0.0;
      sample_recall_[i] = 0.0;
    }
    sum_recall_ = 0.0;
    sum_precision_ = 0.0;

}

RocAccumulator::~RocAccumulator() {
  //Destructor
}

void RocAccumulator::Add(double recall, double precision){
  //Accumulate recall and prevision value
  // std::cout << "[RocAccumultator] Recall: "<< recall << std::endl;
  if (windowsamples_ < N) {
    sample_precision_[windowsamples_] = precision;
    
    sum_precision_ += precision;
    sum_recall_ += recall;

    // std::cout << "[RocAccumultator] SUM Recall: "<< sum_recall_ << std::endl;

    sample_recall_[windowsamples_] = recall;

  } else {
    double &oldest_precision = sample_precision_[windowsamples_ % N];
    double &oldest_recall = sample_recall_[windowsamples_ % N];

    sum_precision_ = sum_precision_ - oldest_precision + precision;
    sum_recall_ = sum_recall_ - oldest_recall + recall;
    oldest_precision = precision;
    oldest_recall = recall;
  }
  windowsamples_++;

  ++totalsamples_;
}

void RocAccumulator::GetMeanRecall(double &meanrecall){
  if(windowsamples_ > N) meanrecall = sum_recall_ / N;
  else meanrecall = sum_recall_ / windowsamples_;
}

void RocAccumulator::GetMeanPrecision(double &meanprecision){
  if(windowsamples_ > N) meanprecision = sum_precision_ / N;
  else meanprecision = sum_precision_ / windowsamples_;
}

int RocAccumulator::GetWindowSamples(){
  return windowsamples_;

}