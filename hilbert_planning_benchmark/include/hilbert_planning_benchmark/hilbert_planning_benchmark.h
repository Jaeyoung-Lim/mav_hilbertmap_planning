//  April/2019, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "mav_planning_benchmark/local_planning_benchmark.h"

namespace mav_planning{
class HilbertPlanningBenchmark : public mav_planning::LocalPlanningBenchmark {
  private:


  public:
    HilbertPlanningBenchmark(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    virtual ~HilbertPlanningBenchmark();

    
};
}
