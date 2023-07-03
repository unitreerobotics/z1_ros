#pragma once

/* Normalised s-curve */
#include <iostream>
#include <chrono>

class SCurve{
public:
    SCurve(){}
    ~SCurve(){}
    void setSCurve(double deltaQ, double dQMax, double ddQMax, double dddQMax);
    void restart();
/*
Taking normalisation into account, in practical use, the physical quantities are
acc = deltaQ * getDDs();
*/
    double getDDs();
    double getDDs(double t);
/*
Taking normalisation into account, in practical use, the physical quantities are
vel = deltaQ * getDs();
*/
    double getDs();
    double getDs(double t);
/*
Taking normalisation into account, in practical use, the physical quantities are
pos = deltaQ * gets();
*/
    double gets();
    double gets(double t);

    double getT();
private:
    int _getSegment(double t);
    void _setFunc();
    double _runTime();

    bool _started = false;
    std::chrono::steady_clock::time_point _startTime;

    double _J, _aMax, _vMax;
    double _T[7];   // period
    double _t[7];   // moment
    double _v0, _v1;        // ds at _t[0], _t[1]
    double _s0, _s1, _s2, _s3, _s4, _s5, _s6;   // s at _t[0], _t[1]
};
