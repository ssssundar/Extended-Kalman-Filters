#include "kalman_filter.h"
#include <math.h>
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;


/**
The minimum value of parameters. Tweak this, if necessary.
*/
#define MIN_VALUE 0.001

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

  cout << "KalmanFilter::Init() - done" << endl;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  //cout << "KalmanFilter::Predict() - enter" << endl;
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;

  cout << "KalmanFilter::Predict():" << P_ << endl;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */

  /**
  Reference: Section 6: Laser Measurements Part 4.
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
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

  cout << "KalmanFilter::Upadte() - done" << endl;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  /**
  Reference: Lesson 6: Section 14. Radar Measurements
  */


  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  //float rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  float rho = sqrt(px*px + py*py);

  if( fabs(px) < MIN_VALUE ) {
    px = MIN_VALUE;
  }
  //float phi = atan2(x_(1), x_(0));
  float phi = atan2(py, px);

  if( fabs(rho) < MIN_VALUE ) {
    rho = MIN_VALUE;
  }  
  // rho_dot = (x_(0)*x_(2) + x_(1)*x_(3)) / rho; 
  float rho_dot = (px*vx + py*vy) / rho;

/**  
if (phi > M_PI) {
    cout << "Big Phi:" << phi << endl;
  }
  else if (phi < -M_PI) {
    cout << "Small Phi:" << phi << endl;
  }
  else {
  **
  Normal value. Do nothing.
  *
    cout << "Phi:" << phi << endl;
  }
*/ 
  
 /** 
  Avoid divide by 0.
  
  if( fabs(rho) < 0.0001 ) {
    rho_dot = 0;
  } else {
    rho_dot = (x_(0)*x_(2) + x_(1)*x_(3)) / rho;
  }
*/

  /**
  Reference: Section 6: Laser Measurements Part 4.
  */

  VectorXd z_pred(3);
  z_pred << rho, phi, rho_dot;
  VectorXd y = z - z_pred;
  if ( fabs(y[1]) > M_PI ) {
    y[1] = atan2(sin(y[1]), cos(y[1]));  
  } 

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
  
  cout << "KalmanFilter::UpdateEKF():" << P_  << endl;
}
