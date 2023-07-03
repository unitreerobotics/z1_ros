#ifndef JOINT_TRAJECTORY_H
#define JOINT_TRAJECTORY_H

#include "UnitreeArmModule/trajectory/QuinticPolynomial.h"
#include <chrono>
#include <memory>

class JointTraj
{
public:
  JointTraj(Vec6 startQ, Vec6 endQ, double maxJointSpeed);
  ~JointTraj(){};

  bool getJointCmd(Vec6& q, Vec6& dq);
  double getPathTime() {return pathTime_;}
private:
  void runTime();

  bool setCorrectly{false};

  /* runtime */
  bool timeReached_ = false;
  double pathTime_{};
  double timeCost_;
  Vec6 startQ_, endQ_;

  std::chrono::steady_clock::time_point startTime_;
  bool pathStarted_ = false;

  QuinticPolynomial<6> quinticPolynomial;
};

#endif // JOINT_TRAJECTORY_H