//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#ifndef ROC_ACCUMULATOR_H
#define ROC_ACCUMULATOR_H

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include "hilbert_mapper/hilbert_mapper.h"
#include <Eigen/Dense>

class RocAccumulator
{
  private:
    double mean_precision_;
    double mean_recall_;
    double threshold_;
    double max_;
    double min_;
    double sum_precision_;
    double sum_recall_;
    int N;
    int windowsamples_;
    int totalsamples_;
    std::vector<double> sample_precision_;
    std::vector<double> sample_recall_;

public:
    RocAccumulator();
    virtual ~ RocAccumulator();
    void Add(double recall, double precision);
    void GetMeanRecall(double &meanrecall);
    void GetMeanPrecision(double &meanprecision);
    int GetWindowSamples();
};

#endif //ROC_ACCUMULATOR_H