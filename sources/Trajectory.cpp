#include "headers/Trajectory.h"


TrajectoryPlanner::TrajectoryPlanner(double dt){
    this->dt_ = dt;
}

MatrixXd TrajectoryPlanner::cubicPoly(double* way_pts, double* vel_pts, double* time_pts, const int pts_count){
    /*
        Returns Third Order Polynomial Trajectory (Position, Velocity & Acceleration) 
        with Respect to the Given Way Points
    */
    int length = 1/dt_*(time_pts[pts_count]-time_pts[0]);
    
    MatrixXd q(1, length);
    MatrixXd qd(1, length);
    MatrixXd qdd(1, length);

    const short coef_dim = 4;
    MatrixXd coef_mat(pts_count, coef_dim);

    for(int i=0; i<pts_count; i++){
        double final_time = time_pts[i+1] - time_pts[i];
        coef_mat.row(i) = Map <Matrix<double , 1, coef_dim>> (this->cubicCoefs(way_pts[i], way_pts[i+1], vel_pts[i], vel_pts[i+1], final_time));
    }

    for(int i=0; i<length; i++){
        double time = i * dt_;
        for(int j=0; j<pts_count; j++){
            if (time < time_pts[j+1] && time >= time_pts[j]){
                double local_time = time-time_pts[j];
                q(i) = coef_mat(j, 0) + coef_mat(j, 1) * local_time + coef_mat(j, 2) * pow(local_time,2) + coef_mat(j, 3) * pow(local_time,3);
                qd(i) = coef_mat(j, 1) + 2 * coef_mat(j, 2) * local_time + 3 * coef_mat(j, 3) * pow(local_time,2);
                qdd(i) = 2 * coef_mat(j, 2) + 6 * coef_mat(j, 2) * local_time;
            }
        }
    }
    MatrixXd output(3, length);
    output << q, qd, qdd;
    return output;
}

double* TrajectoryPlanner::cubicCoefs(double theta_ini, double theta_f, double theta_dot_ini, double theta_dot_f, double tf){
    /* 
        Returns Cubic Polynomial Coefficients with the Given Boundary Conditions
        https://www.tu-chemnitz.de/informatik//KI/edu/robotik/ws2016/lecture-tg%201.pdf
    */
    static double coefs[4]; // a0, a1, a2, a3
    coefs[0] = theta_ini;
    coefs[1] = theta_dot_ini;
    coefs[2] = 3/pow(tf,2) * (theta_f - theta_ini) - 1/tf * (2 * theta_dot_ini + theta_dot_f);
    coefs[3] = -2/pow(tf,3) * (theta_f - theta_ini) + 1/pow(tf,2) * (theta_dot_ini + theta_dot_f);
    return coefs;
}