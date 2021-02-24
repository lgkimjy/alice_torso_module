
#include "alice_torso_module/torso_module_state.h"

namespace alice
{
TorsoModuleState::TorsoModuleState()
{
  count = 0;
  
  mov_time = 2.0;
  smp_time = 0.008;
  dxl_count = 0;
  all_time_steps = int(mov_time / smp_time) + 1;
  
  calc_joint_tra = Eigen::MatrixXd::Zero(all_time_steps, dxl_count+1);
  
}

TorsoModuleState::~TorsoModuleState()
{
}

}
