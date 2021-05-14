#pragma once

#include "cmath"
#include "fstream"
#include "iostream"
#include "math.h"
#include <vector>

class PID{
    public:
        PID(double kp, double ki, double kd, double timeStep);

    private:
        // Controller Gains 
        double kp_;
        double ki_;
        double kd_;


        double dt_;

        double prevoiusError_;   // Controller Previous Error
        double intI_;            // Controller Integrator

        double getOutput(double deiredValue, double currentValue);
};
