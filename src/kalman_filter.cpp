/*
	Extended Kalman Filter Implementation
*/
#include "kalman_filter.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd; // For Matrix Operations
using Eigen::VectorXd; // For Vector Operations

// Constructor
KalmanFilter::KalmanFilter() {}

// Destructor
KalmanFilter::~KalmanFilter() {}

// KF Initialization
void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in, MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in)
{
	cout << "KalmanFilter::Init" << endl;

	// Variable Initialization
	x_ = x_in;
	P_ = P_in; 
	F_ = F_in;
	H_ = H_in;
	R_ = R_in;
	Q_ = Q_in;
}

// KF Predict Step
void KalmanFilter::Predict() {
	cout << "KalmanFilter::Predict()" << endl;
	cout << "x_: " << x_ << endl;
	cout << "F_: " << F_ << endl;
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	cout << "P_: " << P_ << endl;
	cout << "Q_: " << Q_ << endl;
	P_ = F_ * P_ * Ft + Q_;
}

// KF Measurement Update
void KalmanFilter::Update(const VectorXd &z) {
	cout << "KalmanFilter::Update()" << endl;

	cout << "x_: " << x_ << endl;
	cout << "H_: " << H_ << endl;
	VectorXd z_pred = H_ * x_;
	cout << "z_pred: " << z_pred << endl;

	cout << "z: " << z << endl;
	VectorXd y = z - z_pred;
	cout << "y: " << y << endl;
	MatrixXd Ht = H_.transpose();
	cout << "Ht: " << Ht << endl;

	cout << "H_: " << H_ << endl;
	cout << "P: " << P_ << endl;
	cout << "R_: " << R_ << endl;
	MatrixXd S = H_ * P_ * Ht + R_;
	cout << "S: " << S << endl;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;
	cout << "K: " << K << endl;

	//new estimate
	cout << "y: " << y << endl;
	cout << "x_: " << x_ << endl;
	x_ = x_ + (K * y);
	cout << "x_: " << x_ << endl;
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	cout << "P_: " << P_ << endl;
	cout << "I: " << I << endl;
	cout << "K: " << K << endl;
	cout << "H_: " << H_ << endl;
	P_ = (I - K * H_) * P_;
	cout << "P_: " << P_ << endl;
}

// Extended KF Measurement Update
void KalmanFilter::UpdateEKF(const VectorXd &z) {
    cout << "KalmanFilter::UpdateEKF()" << endl;
    cout << "H_: " << H_ << endl;

    VectorXd z_pred = hx_;
    cout << "z_pred: " << z_pred << endl;
    cout << "z: " << z << endl;
    cout << "y = z - z_pred" << endl;
    VectorXd y = z - z_pred;
    
    bool in_pi = false;
    while (in_pi == false) {
      if (y(1) > 3.14159) //pi 
      {
        cout << "phi > pi" << endl;
        y(1) = y(1) - 6.2831; //2*pi
      }
      else if (y(1) < -3.14159) 
      {
        cout << "phi < -pi" << endl;
        y(1) = y(1) + 6.2831; //2*pi
      } 
      else 
      {
        in_pi = true;
      }
    }

    cout << "y: " << y << endl;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;
    cout << "K_: " << K << endl;
    
    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    cout << "I: " << I << endl;
    P_ = (I - K * H_) * P_;
}
