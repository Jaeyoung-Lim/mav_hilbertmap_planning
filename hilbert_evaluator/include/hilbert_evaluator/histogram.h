//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#ifndef HISTOGRAM_H
#define HISTOGRAM_H

#include <vector>

using namespace std;

class Histogram
{
  private:
    std::vector<double> bin_limits_;
    std::vector<double> bin_values_;

  public:
    Histogram(int binsize);
    virtual ~Histogram();
    void add(double value);
};

#endif //ROC_ACCUMULATOR_H