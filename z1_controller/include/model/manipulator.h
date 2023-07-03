#ifndef _MANIPULATOR_H
#define _MANIPULATOR_H

#include "pinocchio/fwd.hpp"
#include "UnitreeArmModule/math/mathTools.h"
#include "UnitreeArmModule/math/quadProgpp/QuadProg++.h"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"

namespace pin = pinocchio;

class Manipulator
{
public:
  Manipulator(pin::Model &);
  virtual ~Manipulator(){};

  void addEE(const pin::Inertia& inertia, const pin::SE3& placement_wrt_mount);
  pinocchio::SE3 getEE();
  void addLoad(const pin::Inertia& inertia);

  void update(const VecX &Q, const VecX& QDot, const VecX& QDDot);
  pinocchio::SE3 computeFK(const VecX &Q, uint bodyId = 6, bool update_kinematics=true);
  virtual bool computeIK(const pinocchio::SE3& Tdes, const Vec6& Qpast, Vec6& Qresult, bool checkInWorkSpace = false);
  pin::Data::Matrix6x computeJacobian(const VecX &Q);
  pin::Data::Matrix6x computeJdot(const VecX &Q, const VecX& QDot);
  VecX computeID(const VecX &Q, const VecX &QDot, const VecX &QDDot, const Vec6& Ftip);
  Eigen::MatrixXd massMatrix(const VecX& Q);
  
  pinocchio::SE3 getEEPlacement();
  bool isApproachPositionLimit(const VecX& Q, double threhold = 0.01);


  pin::Model model;
  pin::Data data, dataTemp;

  const size_t dof = 6;
  pinocchio::FrameIndex ee_index;
protected:
  bool _checkInConfiguration(const VecX& Q);
  const std::string mount_link_name = "mount_link";
  pinocchio::SE3 mount_placement_wrt_joint6_{};
  const std::string ee_link_name = "EE";
  pinocchio::FrameIndex ee_frame_index{}, mount_frame_index{};
  bool hasEE_ = false;

private:
  pinocchio::Inertia endInertia_{}, payload_{};
  pinocchio::SE3 ee_placement_wrt_mount;
};

class ArmModel : public Manipulator 
{
public:
  ArmModel(pin::Model&);
  ~ArmModel(){};
    
  bool computeIK( const pinocchio::SE3& Tdes, 
                  const Vec6& Qpast, 
                  Vec6& Qresult, 
                  bool checkInWorkSpace = false) override; 
  bool checkInSingularity(const Vec6& Q);
  void sloveQP(const Vec6& twist, const Vec6& QPast, Vec6& QDotResult, double dt);
  void addUnitreeGripper();
  bool checkInWorkSpace(const Vec6& Q);
  double getJoint5Distance(const Vec6& Q);
private:
  double _theta3Bias;
};

#endif // _MANIPULATOR_H
