#pragma once

#include "UnitreeArmModule/math/mathTools.h"
#include <algorithm>
#include <iostream>

template <int dim>
class QuinticPolynomial
{
public:
  QuinticPolynomial(){};
  ~QuinticPolynomial(){};

  void setCurve(const Eigen::Matrix<double, dim, 1>& s_0, 
                const Eigen::Matrix<double, dim, 1>& ds_0, 
                const Eigen::Matrix<double, dim, 1>& dds_0,
                const Eigen::Matrix<double, dim, 1>& s_T, 
                const Eigen::Matrix<double, dim, 1>& ds_T, 
                const Eigen::Matrix<double, dim, 1>& dds_T,
                double pathTime);
  double getPathTime() const { return T_; }
  Eigen::Matrix<double, dim, 1> gets(double t);
  Eigen::Matrix<double, dim, 1> getDs(double t);
  Eigen::Matrix<double, dim, 1> getDDs(double t);

private:
  double T_;
  Eigen::MatrixXd a_;
};

template <int dim>
void QuinticPolynomial<dim>::setCurve(const Eigen::Matrix<double, dim, 1>& s_0, 
                                      const Eigen::Matrix<double, dim, 1>& ds_0, 
                                      const Eigen::Matrix<double, dim, 1>& dds_0,
                                      const Eigen::Matrix<double, dim, 1>& s_T, 
                                      const Eigen::Matrix<double, dim, 1>& ds_T, 
                                      const Eigen::Matrix<double, dim, 1>& dds_T,
                                      double pathTime)
{
  a_.resize(6, s_0.size());
  T_ = pathTime;
  Mat6 A;
  A << 1, 0, 0, 0, 0, 0,
       0, 1, 0, 0, 0, 0,
       0, 0, 2, 0, 0, 0, 
       1, T_, pow(T_,2), pow(T_,3), pow(T_,4), pow(T_,5),
       0, 1, 2*T_, 3*pow(T_,2), 4*pow(T_,3), 5*pow(T_,4),
       0, 0, 2, 6*T_, 12*pow(T_,2), 20*pow(T_,3);

  Vec6 B;
  for(size_t i=0; i<s_0.size(); i++)
  {
    B << s_0(i), ds_0(i), dds_0(i), s_T(i), ds_T(i), dds_T(i);
    a_.col(i) = A.partialPivLu().solve(B);
  }
}

template <int dim>
Eigen::Matrix<double, dim, 1> QuinticPolynomial<dim>::gets(double t)
{
  t = clamp(t, 0.0, T_);
  auto result = a_.row(0) + a_.row(1) * t + a_.row(2) * pow(t,2)
           + a_.row(3) * pow(t,3) + a_.row(4) * pow(t,4) + a_.row(5) * pow(t,5);
  return result.transpose();
}

template <int dim>
Eigen::Matrix<double, dim, 1> QuinticPolynomial<dim>::getDs(double t)
{
  t = clamp(t, 0.0, T_);
  auto result = a_.row(1) + 2 * a_.row(2) * t + 3*a_.row(3)*pow(t,2)
           + 4*a_.row(4)*pow(t,3) + 5*a_.row(5) * pow(t,4);
  return result.transpose();
}

template <int dim>
Eigen::Matrix<double, dim, 1> QuinticPolynomial<dim>::getDDs(double t)
{
  t = clamp(t, 0.0, T_);
  auto  result = 2*a_.row(2) + 6*a_.row(3)*t + 12*a_.row(4)*pow(t,2) + 20*a_.row(5)*pow(t,3);
  return result.transpose();
}
