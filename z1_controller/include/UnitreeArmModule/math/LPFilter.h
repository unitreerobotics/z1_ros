#pragma once

#include <vector>
#include <eigen3/Eigen/Dense>

/*
    Low pass filter
*/
class LPFilter{
public:
    LPFilter(double weight, size_t valueCount);
    LPFilter(double samplePeriod, double cutFrequency, size_t valueCount);
    ~LPFilter(){};
    void clear();
    void addValue(double &newValue);
    void addValue(std::vector<double> &vec);

    template <typename T>
    void addValue(Eigen::MatrixBase<T> &eigenVec);
private:
    size_t _valueCount;
    double _weight;
    std::vector<double> _pastValue;
    bool _start;
};
