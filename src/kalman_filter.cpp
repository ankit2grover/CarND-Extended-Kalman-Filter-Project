#include "kalman_filter.h"
#include <iostream>

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
  TODO:
    * predict the state
  */
  x_ = F_* x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
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
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  auto px = x_(0);
	auto py = x_(1);
	auto vx = x_(2);
	auto vy = x_(3);

  auto c1 = px*px+py*py;
  // Calculate rho and phi.
  auto rho = sqrt(c1);
  if (fabs(px) < 0.001) {
    cout << "px value less than 0.001" << endl;
    return;
  }
  // Calculate phi value
  auto param = py/px;
  auto phi = atan(param);
  //Normalize the Phi value.
  // auto pi2 = 2 * PI;
  // auto min = -1 * PI;
  // auto max = PI;
  // if (fabs(pi2) < 0.001) {
  //   cout << "Phi value is less than 0: " << endl;
  //   return;
  // }
  // auto new_phi = (max - min) * ((phi - min) / pi2);
  // phi = phi + new_phi;
  //End of normalized.
  // Calculate rhodot
  auto rhodot = (px*vx + py*vy) / rho;
  VectorXd Hx = VectorXd(3);
  Hx << rho, phi, rhodot;
  auto y = z - Hx;
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
