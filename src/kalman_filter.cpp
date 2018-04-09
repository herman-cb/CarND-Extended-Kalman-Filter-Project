#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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
  TODO: DONE
    * predict the state
  */
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO: DONE
    * update the state by using Kalman Filter equations
  */

    MatrixXd S = H_ * P_ * H_.transpose() + R_;
    MatrixXd K = P_ * H_.transpose() * S.inverse();
    MatrixXd I = MatrixXd::Identity(4, 4);
      
    
    x_ = x_ + K * (z - H_ * x_);
    P_ = (I - K * H_) * P_;
    std::cout << "Measurement update using Laser" << std::endl;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO: DONE?
    * update the state by using Extended Kalman Filter equations
  */
  std::cout << "z = "<< z << std::endl;
  std::cout << "H = "<< H_ << std::endl;
  std::cout << "P = "<< P_ << std::endl;
  std::cout << "R = "<< R_ << std::endl;
  std::cout << "P_ = "<< P_ << std::endl;

  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  std::cout << "S = "<< S << std::endl;
  std::cout << "S inverse = "<< S.inverse() << std::endl;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  MatrixXd I = MatrixXd::Identity(4, 4);

  std::cout << "K = "<< K << std::endl;
    
  x_ = x_ + K * (z - H_ * x_);
  P_ = (I - K * H_) * P_;
  std::cout << "Measurement update using Radar" << std::endl;

}
