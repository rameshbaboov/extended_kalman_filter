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
  TODO: 
    * predict the state
  */
  // use x_  the previous estimate to calculate the current estimate
  cout << "start prediction: Kalman filter prediction" << endl ;
  x_ = F_ * x_ ;
  cout << "calculate FT" << endl ;
  MatrixXd FT = F_.transpose();
  cout << "CALCULATE P" << endl ;
  P_ = F_ * P_ * FT + Q_;
  cout << "END prediction: Kalman filter prediction" << endl ;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  // z - current measurement
  cout << "start measurement update - laser" << endl;
  cout << H_ << "Value of H" << endl;
  cout << x_ << "Value of x" << endl;
  VectorXd y = z - H_ * x_;
  cout << y << "y" << endl;
  MatrixXd HT = H_.transpose();
  cout << HT << "ht Transpose" << endl;
  MatrixXd S = H_ * P_* HT  + R_;
  cout << S << "S" << endl;
  MatrixXd SINV = S.inverse();
  cout << SINV << "SINV" << endl;
  MatrixXd PHT = P_ * HT;
  cout << PHT << "Pht Transpose" << endl;
  MatrixXd K = PHT * SINV;
  cout << K << "KALMAN GAIN" << endl;
  cout << "end measurement update - laser" << endl;
  
  //Calculate the new estimate
  cout << "start measurement update - new estimate laser" << endl;
  x_ = x_ + (K*y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size,x_size);
  P_ = (I - K * H_)* P_;
 cout << "end measurement update - new estimate laser" << endl;
  
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  // calculate state vector 
  cout << "start measurement update -radar" << endl;
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);
  
  double rho = sqrt(px*px + py*py);
  double theta = atan2(py,px);
  double rho_dot = (px*vx + py*vy)/rho;
  
  VectorXd h = VectorXd(3);
  
  h << rho,theta,rho_dot;
  
  VectorXd  y = z - h;
  while ( y(1) > M_PI || y(1) < -M_PI ) {
    if ( y(1) > M_PI ) {
      y(1) -= M_PI;
    } else {
      y(1) += M_PI;
    }
  }
  
  MatrixXd HT = Hj_.transpose();
  MatrixXd S = Hj_ * P_* HT  + R_;
  MatrixXd SINV = S.inverse();
  MatrixXd PHT = P_ * HT;
  MatrixXd K = PHT * SINV;
   cout << "end measurement update -radar" << endl;
  
  //Calculate the new estimate
  cout << "start measurement update -new estimate - radar" << endl;
  x_ = x_ + (K*y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size,x_size);
  P_ = (I - K * Hj_)* P_;
  cout << "end measurement update -new estimate - radar" << endl; 
  
  
  
}

