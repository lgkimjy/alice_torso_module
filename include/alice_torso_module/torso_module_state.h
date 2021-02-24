
#include <eigen3/Eigen/Eigen>
#include "robotis_math/robotis_math.h"

namespace alice
{

class TorsoModuleState
{
  public:
    TorsoModuleState();
    ~TorsoModuleState();
    
    int count;
    int dxl_count;    
    double mov_time;
    double smp_time;
    
    int all_time_steps;
    
    Eigen::MatrixXd calc_joint_tra;
};

}    
