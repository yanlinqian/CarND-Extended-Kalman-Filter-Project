#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
    
    //compute Kalman gain
    MatrixXd K_dot=P_*H_.transpose()*(( H_*P_*H_.transpose()+R_).inverse());
    
    //compute new estimate
    x_ = x_ + K_dot*(z - H_ * x_);
    P_ = P_ - K_dot*H_*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
    
    
    float rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
    float phi = atan2(x_(1), x_(0));
    float rho_dot;
    if (fabs(rho) < 0.0001) {
        rho_dot = 0;
    } else {
        rho_dot = (x_(0)*x_(2) + x_(1)*x_(3))/rho;
    }
    VectorXd z_pred(3);
    z_pred << rho, phi, rho_dot;
    VectorXd y = z - z_pred;
    
    
    //compute Kalman gain
    MatrixXd K_dot=P_*H_.transpose()*(( H_*P_*H_.transpose()+R_).inverse());
    
    //compute new estimate
    x_ = x_ + K_dot*y;
    P_ = P_ - K_dot*H_*P_;
}
