//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "hilbert_planning_benchmark/hilbert_planning_benchmark.h"

using namespace std;

namespace mav_planning{


HilbertPlanningBenchmark::HilbertPlanningBenchmark(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : LocalPlanningBenchmark(nh, nh_private) {

}

HilbertPlanningBenchmark::~HilbertPlanningBenchmark(){
    //Destructor
}

}