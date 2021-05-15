# pragma once

#include "headers/MinJerk.h"

class Ankle: private MinJerk{
    public:
        Ankle(double step_time, double ds_time, double alpha, short int num_step);
        Vector3d* getTrajectoryL();
        Vector3d* getTrajectoryR();

    private:
        double tStep_;
        double tDS_;
        double dt_;
        short int num_step;
        double alpha;
        double stepCount;
        bool leftFirst;

        Vector3d* footPose_;
        Vector3d* lFoot_;
        Vector3d* rFoot_;

        void updateFoot(Vector3d foot_pose[]);
        void generateTrajectory();        
        void updateTrajectory(bool left_first);
};