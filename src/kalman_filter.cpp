#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

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
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO: DONE
    * update the state by using Kalman Filter equations
  */
    std::cout << "(kalman_filter.cpp) Measurement update using Laser" << std::endl;

    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;

    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

    x_ = x_ + (K * y);
    MatrixXd I = MatrixXd::Identity(4, 4);
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO: DONE?
    * update the state by using Extended Kalman Filter equations
  */
  std::cout << "(kalman_filter.cpp)***** Measurement update using Radar *****" << std::endl;

  float rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  float phi = atan2(x_(1), x_(0));
  float rho_dot;

  if (fabs(rho) < 0.0001) {
      rho_dot = 0.0;
  } else {
      rho_dot = (x_(0) * x_(2) + x_(1) * x_(3))/rho;
  }

  VectorXd z_pred = VectorXd(3);

  z_pred << rho, phi, rho_dot;

  VectorXd y = z - z_pred;

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();

  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  MatrixXd I = MatrixXd::Identity(4, 4);
  x_ = x_ + (K * y);
  P_ = (I- K * H_) * P_;

  cout << "(kalman_filter.cpp) z_pred = " << endl << z_pred << endl;
}
