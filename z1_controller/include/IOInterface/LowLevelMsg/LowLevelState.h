#ifndef _LOW_LEVEL_STATE
#define _LOW_LEVEL_STATE

#include "UnitreeArmModule/math/mathTypes.h"
#include <iostream>
#include "UnitreeArmModule/math/LPFilter.h"
#include <memory>
#include "sdk/unitree_arm_common.h"

/**
 * @brief 包含臂和手爪
 */
struct LowLevelState
{
public:
  LowLevelState(double dt);
  ~LowLevelState();
  
  std::array<double, 7> q{};
  std::array<double, 7> dq{};
  std::array<double, 7> tau{};
  std::array<int8_t, 8> temperature{};
  std::array<uint8_t, 8> errorstate{};
  std::array<uint8_t, 8> isMotorConnected{};
  std::array<bool, UNITREE_ARM_SDK::ErrorNum> errors{};
  std::array<std::array<bool, 8>, 8> motor_errors{};
  
  std::vector<double> qFiltered;
  std::vector<double> dqFiltered;

  Vec6 getQ();
  Vec6 getQFiltered();
  Vec6 getDq();
  Vec6 getDqFiltered();
  Vec6 getTau();

  std::string formattedOutput(int index) const;
  void runFilter();
  bool checkError();

  bool has_motor_errors() const;
private:
  LPFilter *qFilter;
  LPFilter *dqFilter;

  std::array<int, 8> _isMotorConnectedCnt{};
  const int TemporatureLimit = 80; // centigrade
};

#endif // _LOW_LEVEL_STATE