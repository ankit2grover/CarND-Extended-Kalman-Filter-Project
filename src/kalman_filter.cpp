#include "kalman_filter.h"
#include <iostream>
#include "tools.h"

using namespace std;
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
    * predict the state
  */
  x_ = F_* x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
    * update the state by using Kalman Filter equations
  */
  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
    * update the state by using Extended Kalman Filter equations
  */
  auto px = x_(0);
	auto py = x_(1);
	auto vx = x_(2);
	auto vy = x_(3);

  if (fabs(px) < 0.00001) {
    px = 0.00001;
  }

  if (fabs(py) < 0.00001) {
    py = 0.00001;
  }

  double c1 = px*px+py*py;
  
  // Calculate rho.
  double rho = sqrt(c1);
  
  // Calculate bearing(phi) value
  //double phi = atan((py/px));
  double phi = atan2(py, px);
  //double phi = atan2(sin(py), cos(px));
  // Calculate rhodot
  double rhodot;
  // Check if rho value is equal to 0 then reset it and reset rhodot value also.
  if (fabs(rho) < 0.00001) {
    rhodot = 0.0  ;
  } else {
    rhodot = (px*vx + py*vy) / rho;
  } 

  VectorXd Hx = VectorXd(3);
  Hx << rho, phi, rhodot;
  VectorXd y = z - Hx;
  y(1) = atan2(sin(y(1)), cos(y(1)));
  // Another approach to limit the values in [-Pi, Pi]
  //y(1) = tools.wrapMinMax(y(1), -M_PI, M_PI);
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
