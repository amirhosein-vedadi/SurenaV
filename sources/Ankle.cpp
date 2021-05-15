#include "headers/Ankle.h"

AnklePlanner::AnklePlanner(Vector3d* initFP, Vector3d* finFP, double* initT, double* finT, 
                        double swingHeight, int swingCount, double dt) : TrajectoryPlanner(dt) {
    
    this->initFP_ = initFP;
    this->finFP_ = finFP;
    this->initT_ = initT;
    this->finT_ = finT;
    this->swingHeight_ = swingHeight;
    this->swingCount_ = swingCount; 
}

MatrixXd* AnklePlanner::getSwing(){
    /*
        Returns All Swing Foot Trajectories in an Array of MatrixXd
    */
    MatrixXd* swing_trajs = new MatrixXd[swingCount_]; //Array of all swing trajectories
    for(int i=0; i<swingCount_; i++){
        for(int j=0; j<2; j++){
            double time_pts[] = {initT_[i], finT_[i]};
            double way_pts[] = {initFP_[i](j), finFP_[i](j)};
            double vel_pts[] = {0.0, 0.0};
            swing_trajs[i] = TrajectoryPlanner::cubicPoly(way_pts, vel_pts, time_pts, 1);
        }
        double mid_time = (initT_[i] + finT_[i]) / 2;
        double time_pts[] = {initT_[i], mid_time, finT_[i]};
        double way_pts[] = {initFP_[i](2), swingHeight_, finFP_[i](2)};
        double vel_pts[] = {0.0, 0.0, 0.0};
        swing_trajs[i] = TrajectoryPlanner::cubicPoly(way_pts, vel_pts, time_pts, 2);

    }
    return (swing_trajs);
}