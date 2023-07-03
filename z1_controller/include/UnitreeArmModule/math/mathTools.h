#pragma once

#include <iostream>
#include <algorithm>
#include <iomanip>
#include "UnitreeArmModule/math/mathTypes.h"

inline double sign(const double& x, double threhold = 1e-6){
    return std::fabs(x) < threhold ? 0 : ( x>0?1:-1 );
}

inline double cosine_throrem(double a, double b, double c) {
    // c^2 = a^2 + b^2 -2ab*cos(gamma)
    return std::acos( (std::pow(a,2)+std::pow(b,2)-std::pow(c,2))/(2*a*b));
};

inline double clamp(const double& x, const double& low_value, const double& high_value) {
    return (x > low_value) ? (x<high_value?x:high_value) : low_value;
}

inline bool inInterval(double value, double lowLim, double highLim, double tolerance = 0.0){
    return (value < highLim + tolerance) && (value > lowLim - tolerance);
}

inline bool inInterval(VecX value, VecX lowLim, VecX highLim, double tolerance = 0.0) {
    return ( (lowLim.array() - tolerance <= value.array()).all() &&
             (value.array() <= highLim.array() + tolerance).all() );
}

inline double isVectorTowardsPoint(const Vec3& vec, const Vec3& point) {
    return vec.dot(point) / (vec.norm() * point.norm() );
}