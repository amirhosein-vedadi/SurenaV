#pragma once

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"

#include "cmath"
#include "fstream"
#include "iostream"
#include "math.h"
#include <vector>

using namespace Eigen;
using namespace std;


class _Link{
    friend class Robot;
    public:
        _Link(short int ID, string name, short int parentID, short int numChilds, short int* childID, double length);
        short int getID();
        double q();
        void q(double config);
        short int* getChildID();
        short int getParentID();
        
    private:
        short int ID_;
        Vector3d worldAttitude;     // Rotation relative to world frame
        Vector3d worldPose;         // Pose Relative to world frame
        double q_;                  // Link config (theta in DH)
        short int* childID_;
        short int parentID_;

        string name_;
        double length_;
};