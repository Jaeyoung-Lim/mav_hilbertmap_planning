//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#ifndef HISTOGRAM_H
#define HISTOGRAM_H

#include <vector>
#include <stdio.h>
#include <iostream>

using namespace std;

class Histogram
{
  private:
    int binsize_;
    double width_;
    std::vector<double> bin_limits_;
    std::vector<int> bin_values_;

  public:
    Histogram(int binsize, double width);
    virtual ~Histogram();
    void add(double value);
    void Save();
    void PrintValues();
};

#endif //ROC_ACCUMULATOR_H