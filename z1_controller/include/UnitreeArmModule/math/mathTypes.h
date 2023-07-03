#pragma once

#include <eigen3/Eigen/Dense>
#include <vector>

/************************/
/******** Vector ********/
/************************/
using Vec2 = typename Eigen::Matrix<double, 2, 1>;
using Vec3 = typename Eigen::Matrix<double, 3, 1>;
using Vec4 = typename Eigen::Matrix<double, 4, 1>;
using Vec6 = typename Eigen::Matrix<double, 6, 1>;
using Vec7 = typename Eigen::Matrix<double, 7, 1>;

// Quaternion
using Quat = typename Eigen::Matrix<double, 4, 1>;

// 4x1 Integer Vector
using VecInt4 = typename Eigen::Matrix<int, 4, 1>;

// 12x1 Vector
using Vec12 = typename Eigen::Matrix<double, 12, 1>;

// 18x1 Vector
using Vec18 = typename Eigen::Matrix<double, 18, 1>;

// Dynamic Length Vector
using VecX = typename Eigen::Matrix<double, Eigen::Dynamic, 1>;

/************************/
/******** Matrix ********/
/************************/
// Rotation Matrix
using RotMat = typename Eigen::Matrix<double, 3, 3>;

// Homogenous Matrix
using HomoMat = typename Eigen::Matrix<double, 4, 4>;

// 2x2 Matrix
using Mat2 = typename Eigen::Matrix<double, 2, 2>;

// 3x3 Matrix
using Mat3 = typename Eigen::Matrix<double, 3, 3>;

// 4x4 Matrix
using Mat4 = typename Eigen::Matrix<double, 4, 4>;

// 3x4 Matrix, each column is a 3x1 vector
using Vec34 = typename Eigen::Matrix<double, 3, 4>;

// 6x6 Matrix
using Mat6 = typename Eigen::Matrix<double, 6, 6>;

// 12x12 Matrix
using Mat12 = typename Eigen::Matrix<double, 12, 12>;

// Dynamic Size Matrix
using MatX = typename Eigen::Matrix<double, -1, -1>;

template<typename T>
inline VecX stdVecToEigenVec(T stdVec){
    VecX eigenVec = Eigen::VectorXd::Map(&stdVec[0], stdVec.size());
    return eigenVec;
}

inline std::vector<double> EigenVectostdVec(VecX eigenVec){
    std::vector<double> stdVec;
    for(int i(0); i<eigenVec.size();i++){
        stdVec.push_back(eigenVec(i));
    }
    return stdVec;
}
