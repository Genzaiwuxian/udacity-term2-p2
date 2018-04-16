#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.8;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.30;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  //laser measurement covariance matrix
  R_laser_ = MatrixXd(2, 2);
  R_laser_ << std_laspx_ * std_laspx_, 0,
	  0, std_laspy_*std_laspy_;

  //radar measurement covariance matrix
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << std_radr_ * std_radr_, 0, 0,
	  0, std_radphi_*std_radphi_, 0,
	  0, 0, std_radrd_*std_radrd_;

  //havn't initialized yet
  is_initialized_ = false;

  //current time set as 0
  time_us_ = 0.0;

  //dimension of state is 5: px, py, v, phi, phi_dot
  n_x_ = 5;

  //two augmented state: std_a_ and std_yawdd_
  n_aug_ = 7;

  // tuning parameters, as udacity lession said, lambda choose 3_n_x_ will get a good result
  lambda_ = 3 - n_x_;

  // lambda for augmentation
  lambda_aug = 3 - n_aug_;

  //weights for calculating x_k+1 and P_k+1
  weights_ = VectorXd(2 * n_aug_ + 1);

  double weights_0 = lambda_aug / (lambda_aug + n_aug_);

  weights_(0) = weights_0;

  for (int i = 1; i < 2 * n_aug_ + 1; ++i)
  {
	  double weights = 0.5 / (lambda_aug + n_aug_);
	  weights_(i) = weights;
  }
 
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  NIS_laser = 0.0;

  NIS_radar = 0.0;

  NIS_laser_total = 0.0;

  NIS_radar_total = 0.0;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
	if (!is_initialized_)
	{
		cout << "UKF: " << endl;
		x_ << 1.0, 1.0, 1.0, 1.0, 0.1;
		if (meas_package.sensor_type_ == MeasurementPackage::LASER)
		{
			float p_x, p_y;
			p_x = meas_package.raw_measurements_(0);
			p_y = meas_package.raw_measurements_(1);
			x_(0) = p_x;
			x_(1) = p_y;

			P_ << 1.0, 0.0, 0.0, 0.0, 0.0,
				0.0, 1.0, 0.0, 0.0, 0.0,
				0.0, 0.0, 0.1, 0.0, 0.0,
				0.0, 0.0, 0.0, 0.1, 0.0,
				0.0, 0.0, 0.0, 0.0, 0.1;
		}
		else if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
		{
			float rho, theta, rho_dot;
			rho = meas_package.raw_measurements_(0);
			theta = meas_package.raw_measurements_(1);
			rho_dot = meas_package.raw_measurements_(2);
			x_(0) = rho * cos(theta);
			x_(1) = rho * sin(theta);
			x_(2) = rho_dot;

			P_<< 1.0, 0.0, 0.0, 0.0, 0.0,
				0.0, 1.0, 0.0, 0.0, 0.0,
				0.0, 0.0, 0.1, 0.0, 0.0,
				0.0, 0.0, 0.0, 0.1, 0.0,
				0.0, 0.0, 0.0, 0.0, 0.1;
		}
		time_us_ = meas_package.timestamp_;

		is_initialized_ = true;
		cout << "x_ 1st prediction is: " << endl;
		cout << x_ << endl;
		cout << "P_ 1st prediction is: " << endl;
		cout << P_ << endl;
		cout << "____________________" << endl;
		return;
	}

	float delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
	time_us_ = meas_package.timestamp_;
	
	//cout << "dt: " << endl;
	//cout << delta_t << endl;

	Prediction(delta_t);

	if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_)
	{
		UpdateRadar(meas_package);
	}
	else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_)
	{
		UpdateLidar(meas_package);
	}

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
	// generating sigma points
	MatrixXd A = P_.llt().matrixL();
	MatrixXd Xsig = MatrixXd(n_x_, 2 * n_x_ + 1);

	Xsig.col(0) = x_;
	for (unsigned int i = 1; i < n_x_ + 1; ++i)
	{
		Xsig.col(i) = x_ + sqrt(lambda_ + n_x_)*A.col(i-1);
		Xsig.col(i + n_x_) = x_ - sqrt(lambda_ + n_x_)*A.col(i-1);
	}

	// generating augumented points
	VectorXd x_aug = VectorXd(n_aug_);
	MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
	MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

	x_aug.head(5) = x_;
	x_aug(5) = 0;
	x_aug(6) = 0;
	//cout << "prediction: x_aug: " << endl;
	//cout << x_aug << endl;


	P_aug.fill(0.0);
	P_aug.topLeftCorner(5, 5) = P_;
	P_aug(5, 5) = std_a_ * std_a_;
	P_aug(6, 6) = std_yawdd_ * std_yawdd_;
	MatrixXd L = P_aug.llt().matrixL();
	//cout << "prediction: P_aug: " << endl;
	//cout << P_aug << endl;


	Xsig_aug.col(0) = x_aug;
	for (unsigned int i = 0; i < n_aug_; ++i)
	{
		Xsig_aug.col(i+1) = x_aug + sqrt(lambda_aug + n_aug_)*L.col(i);
		Xsig_aug.col(i + n_aug_+1) = x_aug - sqrt(lambda_aug + n_aug_)*L.col(i);
	}
	//cout << "prediction: Xsig_aug: " << endl;
	//cout << Xsig_aug << endl;


	//sigma point prediction assignment
	Xsig_pred_.fill(0.0);
	for (unsigned int i = 0; i < 2 * n_aug_ + 1; ++i)
	{
		double p_x = Xsig_aug(0, i);
		double p_y = Xsig_aug(1, i);
		double v = Xsig_aug(2, i);
		double yaw = Xsig_aug(3, i);
		double yaw_dot = Xsig_aug(4, i);
		double std_a = Xsig_aug(5, i);
		double std_yaw = Xsig_aug(6, i);

		double px_p, py_p, v_p, yaw_p, yawd_p;

		if (fabs(yaw_dot) < 0.001)
		{
			px_p = p_x + v * cos(yaw)*delta_t + 0.5*delta_t*delta_t*cos(yaw)*std_a;
			py_p = p_y + v * sin(yaw)*delta_t + 0.5*delta_t*delta_t*sin(yaw)*std_a;
		}
		else
		{
			px_p = p_x + (v / yaw_dot)*(sin(yaw + yaw_dot * delta_t) - sin(yaw)) + 0.5*delta_t*delta_t*cos(yaw)*std_a;
			py_p = p_y + (v / yaw_dot)*(-cos(yaw + yaw_dot * delta_t) + cos(yaw)) + 0.5*delta_t*delta_t*sin(yaw)*std_a;
		}
		v_p = v + delta_t * std_a;
		yaw_p = yaw + yaw_dot * delta_t + 0.5*delta_t*delta_t*std_yaw;
		yawd_p = yaw_dot + delta_t * std_yaw;

		Xsig_pred_(0, i) = px_p;
		Xsig_pred_(1, i) = py_p;
		Xsig_pred_(2, i) = v_p;
		Xsig_pred_(3, i) = yaw_p;
		Xsig_pred_(4, i) = yawd_p;
	}
	//cout << "prediction: Xsig_pred_: " << endl;
	//cout << Xsig_pred_ << endl;

	//predicted mean and covariance
	x_.fill(0.0);
	//cout << "prediction: weights" << endl;
	//cout << weights_ << endl;

	for (unsigned int i = 0; i < 2 * n_aug_ + 1; ++i)
	{
		x_ = x_ + weights_(i)*Xsig_pred_.col(i);
	}

	P_.fill(0.0);
	for (unsigned int i = 0; i < 2 * n_aug_ + 1; ++i)
	{
		VectorXd diff = VectorXd(n_x_);
		diff = Xsig_pred_.col(i) - x_;
		if (diff(3) < -M_PI)
			diff(3) += 2 * M_PI;
		else if (diff(3) > M_PI)
			diff(3) -= 2 * M_PI;

		P_ = P_ + weights_(i)*diff*diff.transpose();
	}
	//cout << "x_ prediction is: " << endl;
	//cout << x_ << endl;
	//cout << "P_ prediction is: " << endl;
	//cout << P_ << endl;
	//cout << "____________________" << endl;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
	VectorXd x_pre = VectorXd(n_x_);
	x_pre = x_;

	int n_z = 2;
	MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
	Zsig.fill(0.0);
	for (unsigned int i = 0; i < 2 * n_aug_ + 1; ++i)
	{
		double px, py;
		px = Xsig_pred_(0, i);
		py = Xsig_pred_(1, i);

		Zsig.col(i) << px, py;
	}

	//mean prediction measurement
	VectorXd Z_pred = VectorXd(n_z);
	Z_pred.fill(0.0);
	for (unsigned int i = 0; i < 2 * n_aug_ + 1; ++i)
	{
		Z_pred += weights_(i)*Zsig.col(i);
	}

	//innovation covariance matrix S
	MatrixXd S = MatrixXd(n_z, n_z);
	S.fill(0.0);
	for (unsigned int i = 0; i < 2 * n_aug_ + 1; ++i)
	{
		VectorXd z_diff = VectorXd(n_z);

		z_diff = Zsig.col(i) - Z_pred;
		S += weights_(i)*z_diff*z_diff.transpose();
	}
	S = S + R_laser_;

	//create matrix for cross correlation Tc
	MatrixXd Tc = MatrixXd(n_x_, n_z);
	Tc.fill(0.0);
	for (unsigned int i = 0; i < 2 * n_aug_ + 1; ++i)
	{
		VectorXd z_diff = VectorXd(n_z);
		z_diff = Zsig.col(i) - Z_pred;

		VectorXd x_diff = VectorXd(n_x_);
		x_diff = Xsig_pred_.col(i) - x_;

		Tc += weights_(i)*x_diff*z_diff.transpose();
	}

	//kalman gain K
	MatrixXd K = MatrixXd(n_x_, n_z);
	K = Tc * S.inverse();

	//residual
	VectorXd z_diff = VectorXd(n_z);
	z_diff = meas_package.raw_measurements_ - Z_pred;

	//update state mean and covirance
	x_ = x_ + K * z_diff;
	P_ = P_ - K * S*K.transpose();

	//check radar rmse, drop the fault value


	//Normalized innovation squared (NIS)
	double NIS;
	NIS = z_diff.transpose()*S.inverse()*z_diff;
	NIS_laser_total=NIS_laser_total+1.0;
	if (NIS > 5.991)
	{
		NIS_laser = NIS_laser + 1.0;
		if (NIS_laser_total > 0.00001)
		{
			float percent_NIS_laser = (NIS_laser / NIS_laser_total) * 100;
			cout << percent_NIS_laser << "% Lindar NIS >5.991" << endl;
		}
	}

	//cout << "NIS is: " << NIS << endl;
	//cout << "x_ laser update is: " << endl;
	//cout << x_ << endl;
	//cout << "P_ laser update is: " << endl;
	//cout << P_ << endl;
	//cout << "____________________" << endl;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
	//predict radar measurment assignments
	VectorXd x_pre = x_;

	
	int n_z = 3;
	MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
	Zsig.fill(0.0);
	for (unsigned int i = 0; i < 2 * n_aug_ + 1; ++i)
	{
		double px, py, v, yaw, rho, theta, rho_dot;
		px = Xsig_pred_(0, i);
		py = Xsig_pred_(1, i);
		v = Xsig_pred_(2, i);
		yaw = Xsig_pred_(3, i);

		rho = sqrt(px*px + py * py);
		if (fabs(px) < 1.0e-8)
		{
			px = 1.0e-8;
		}
		theta = atan2(py, px);
		if (fabs(rho) < 1.0e-8)
		{
			rho = 1.0e-8;
		}
		rho_dot = (px*cos(yaw)*v + py * sin(yaw)*v) / rho;

		Zsig.col(i) << rho, theta, rho_dot;
	}

	//mean prediction measurement
	VectorXd Z_pred = VectorXd(n_z);
	Z_pred.fill(0.0);
	for (unsigned int i = 0; i < 2 * n_aug_ + 1; ++i)
	{
		Z_pred += weights_(i)*Zsig.col(i);
	}

	//innovation covariance matrix S
	MatrixXd S = MatrixXd(n_z, n_z);
	S.fill(0.0);
	for (unsigned int i = 0; i < 2 * n_aug_ + 1; ++i)
	{
		VectorXd z_diff = VectorXd(n_z);

		z_diff = Zsig.col(i) - Z_pred;
		if (z_diff(1) < -M_PI)
			z_diff(1) += 2 * M_PI;
		else if (z_diff(1) > M_PI)
			z_diff(1) -= 2 * M_PI;

		S += weights_(i)*z_diff*z_diff.transpose();
	}

	S = S + R_radar_;
	//create matrix for cross correlation Tc
	MatrixXd Tc = MatrixXd(n_x_, n_z);
	Tc.fill(0.0);
	for (unsigned int i = 0; i < 2 * n_aug_ + 1; ++i)
	{
		VectorXd z_diff = VectorXd(n_z);
		z_diff = Zsig.col(i) - Z_pred;
		if (z_diff(1) < -M_PI) z_diff(1) += 2 * M_PI;
		else if (z_diff(1) > M_PI) z_diff(1) -= 2 * M_PI;

		VectorXd x_diff = VectorXd(n_x_);
		x_diff = Xsig_pred_.col(i) - x_;
		if (x_diff(3) < -M_PI) x_diff(3) += 2 * M_PI;
		else if (x_diff(3) > M_PI) x_diff(3) -= 2 * M_PI;

		Tc += weights_(i)*x_diff*z_diff.transpose();
	}

	//kalman gain K
	MatrixXd K = MatrixXd(n_x_, n_z);
	K = Tc * S.inverse();

	//residual
	VectorXd z_diff = VectorXd(n_z);
	z_diff = meas_package.raw_measurements_ - Z_pred;
	if (z_diff(1) < -M_PI) z_diff(1) += 2 * M_PI;
	else if (z_diff(1) > M_PI) z_diff(1) -= 2 * M_PI;

	//update state mean and covirance
	x_ = x_ + K * z_diff;
	P_ = P_ - K * S*K.transpose();

	//check px&py
	VectorXd rmse_xy = VectorXd(2);
	rmse_xy(0) = fabs(x_pre(0) - x_(0));
	rmse_xy(1) = fabs(x_pre(1) - x_(1));
	//cout << "rmse_xy: " << endl;
	//cout << rmse_xy << endl;

	if (rmse_xy(0) > 2.0 || rmse_xy(1) > 2.0)
	{
		x_ = x_pre;
		return;
	}

	//Normalized innovation squared (NIS)
	double NIS;
	NIS = z_diff.transpose()*S.inverse()*z_diff;
	NIS_radar_total=NIS_radar_total+1.0;
	if (NIS > 7.815)
	{
		NIS_radar = NIS_radar + 1.0;
		if (NIS_radar_total > 0.00001)
		{
			float percent_NIS_radar = (NIS_radar / NIS_radar_total) * 100;
			cout << percent_NIS_radar << "% Radar NIS >7.815" << endl;
		}
	}


	//cout << "NIS is: " << NIS << endl;
	//cout << "x_ radar update is: " << endl;
	//cout << x_ << endl;
	//cout << "P_ radar update is: " << endl;
	//cout << P_ << endl;
	//cout << "____________________" << endl;
}
