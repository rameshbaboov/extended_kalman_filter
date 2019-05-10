#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  ekf_.Hj_ = MatrixXd(3, 4);

  ekf_.x_ = VectorXd(4);
  ekf_.F_ = MatrixXd(4,4);
  ekf_.P_ = MatrixXd(4,4);
  
  // set matrices to values
  ekf_.P_ << 1,0,0,0,
		0,1,0,0,
		0,0,1000,0,
		0,0,0,1000;

  H_laser_ <<1,0,0,0,
             0,1,0,0;
  ekf_.F_  << 1,0,1,0,
	      0,1,0,1,
	      0,0,1,0,
	      0,0,0,1;		 	
  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;
  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

 }

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initializationcalculate
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
	
    cout << "EKF: Initialization" << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.H_ = MatrixXd(2,4);
    ekf_.R_ = MatrixXd(2,2);
    ekf_.x_ << 1, 1, 1, 1;
	
	
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
	// added for debug
	//return;
	//added for debug	
      cout << "initializing RADAR measurement with first measurement" << endl;
	  /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
	 	 
	  double rho = measurement_pack.raw_measurements_[0];  //range
	  double phi = measurement_pack.raw_measurements_[1];  //bearing
	  double rho_dot = measurement_pack.raw_measurements_[2];  //rho velocity
	  double px = rho * cos(phi);
	  double py = rho * sin(phi);
	  double vx = rho_dot * cos(phi);
	  double vy = rho_dot * sin(phi);
	  
	  if (px < 0.0001) { px = 0.0001; }
	  if (py < 0.0001) { py = 0.0001; }
	  
	  ekf_.x_ << px,py,vx,vy;
	   
	  
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
     // added for debug
	//return;
	//added for debug	
      /**
      Initialize state.
      */
	  ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
	  
    }

    // done initializing, no need to predict or updateccalculatealculate FT

    previous_timestamp_ = measurement_pack.timestamp_;
	is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.calculate FT

   */
	// cde added for debug
	// if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) { return;}
	// if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) { return;}
	//code added for dbug
        
	//compute the time elapsed between the current and previous measurements
	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
	previous_timestamp_ = measurement_pack.timestamp_;
	float dt_2 = dt * dt;
	float dt_3 = dt_2 * dt;
	float dt_4 = dt_3 * dt;
	double noise_ax = 9;
        double noise_ay = 9;
	ekf_.F_(0, 2) = dt;
	ekf_.F_(1, 3) = dt;
        

	//set the process covariance matrix Q
	ekf_.Q_ = MatrixXd(4, 4);
	ekf_.Q_  <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
			   0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
			   dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
			   0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

  cout <<"starting prediction"<<endl; 

  ekf_.Predict();
  cout <<"prediction done"<<endl;
  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */
   

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
	cout   <<"entering radar measurement updates" << endl;
	cout << " calculating Jacobian matrix with x_ " << ekf_.x_ << endl;
	ekf_.Hj_ = tools.CalculateJacobian(ekf_.x_);
	cout <<" Jacobian calculated " << endl;
	ekf_.R_ = R_radar_;
	cout <<" performing update function" << endl;
	ekf_.UpdateEKF(measurement_pack.raw_measurements_);
	cout << " performed update function";
  } else {
    // Laser updates
	cout <<" entering laser updates. performing update" << endl;
        cout << H_laser_ << "Value of H_laser_" << endl;
       ekf_.H_ = H_laser_;
       cout << R_laser_ << "Value of R_laser_" << endl;
       ekf_.R_ = R_laser_;
       cout << ekf_.H_ << "Value of H" << endl;
       cout << ekf_.x_ << "Value of x" << endl;
       cout << ekf_.R_ << "Value of R" << endl;


    ekf_.Update(measurement_pack.raw_measurements_);
	cout << " performed update function";

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
