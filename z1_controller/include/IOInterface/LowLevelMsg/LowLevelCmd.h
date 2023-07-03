#ifndef _LOW_LEVEL_CMD_H
#define _LOW_LEVEL_CMD_H

#include <array>
#include "UnitreeArmModule/math/mathTypes.h"
#include <iostream>

struct LowLevelCmd
{
  std::array<double, 7> q{};
  std::array<double, 7> dq{};
  std::array<double, 7> tau{};
  std::array<double, 7> Kp{};
  std::array<double, 7> Kd{};
  
  Vec6 getKp();
  Vec6 getKd();
  Vec6 getQ();
  Vec6 getDq();
  Vec6 getTau();

  void setDefaultGain();
  void setPassive();
  void setQ(const Vec6& qInput);
  void setDq(const Vec6& dqInput);
  void setTau(const Vec6& tauInput);
  std::string formattedOutput(int index) const;
};


#endif // _LOW_LEVEL_CMD_H