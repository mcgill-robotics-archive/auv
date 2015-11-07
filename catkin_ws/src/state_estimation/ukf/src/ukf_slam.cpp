#include <angles/angles.h>
#include <boost/bind.hpp>
#include <cmath>
#include <cstdio>
#include "ukf.h"
#include "ukf_slam.h"

ukf_slam::ukf_slam(int n):
  estimator(VectorXd::Zero(3*n), 100 * MatrixXd::Identity(3*n,3*n)),
  N(n)
{}

double lnRange(Vector3d v) {
  return log(std::max(v(0)*v(0) + v(1)*v(1) + v(2)*v(2), 1e-16))/2;
}

double bearing(Vector3d v) {
  return atan2(v(1), v(0));
}

double elevation(Vector3d v) {
  return atan2(v(2), sqrt(v(0)*v(0) + v(1)*v(1)));
}

MatrixXd ukf_slam::observe(int objectIndex, double base_yaw, const Affine3d transform, const MatrixXd sigmas) {
  // Convert states to the sensor frame. Constructing Matrix3Xd is necessary to
  // avoid a compile error on matrix sizes.
  MatrixXd conv_sigmas = transform * Matrix3Xd(sigmas.block(objectIndex, 0, 3, sigmas.cols()));

  // Three rows because ln_range, bearing, elevation
  MatrixXd msmts = MatrixXd(3, conv_sigmas.cols());
  for (int i = 0; i < conv_sigmas.cols(); i++) {
    const Vector3d sigma = conv_sigmas.col(i);
    msmts(0, i) = lnRange(sigma);
    //Restricting the bearing to the range [-pi,pi) centered on base_yaw
    msmts(1, i) = angles::normalize_angle(bearing(sigma) - base_yaw);
    msmts(2, i) = elevation(sigma);
  }
  return msmts;
}

MatrixXd ukf_slam::observe_depth(/*const Affine3d transform,*/ const MatrixXd sigmas)
{
	//Take into account positioning of depth sensor relative to center of gravity of the robot ?
	//^ Maybe later	
	return sigmas.row(2);
}

void ukf_slam::propogate(Ref<Eigen::VectorXd> state) {
  // Does nothing because we don't have odometry implemented yet
}

// Increases the size of the state vector and covariance by dim
void ukf_slam::append(int dim) {
  VectorXd newState(estimator.state.size() + dim);
  newState << estimator.state, VectorXd::Zero(dim);

  MatrixXd newCovar = MatrixXd::Zero(estimator.covariance.rows() + dim,
      estimator.covariance.cols() + dim);
  newCovar.block(0,0, estimator.covariance.rows(), estimator.covariance.cols())
    = estimator.covariance;
  newCovar.block(estimator.covariance.rows(), estimator.covariance.cols(), 
      dim, dim) = 1e15 * MatrixXd::Identity(dim, dim);

  estimator.state.resize(newState.rows() ,newState.cols());
  estimator.state = newState;
  estimator.covariance.resize(newCovar.rows(), newCovar.cols());
  estimator.covariance = newCovar;
  N++;
}

VectorXd ukf_slam::update(int objectIndex, const Affine3d transform, const Vector3d msmt, const Vector3d covar)
{
  estimator.predict(&propogate, 0.01 * MatrixXd::Identity(3*N,3*N));
  //Note that we pass in zero for the measured bearing. This is because observe take in the measured bearing and centers
  // all the sigma point's bearings around the measured bearing so we don't run into issues with the jump from pi to -pi
  estimator.correct(Vector3d(msmt(0), 0, msmt(2)),
    boost::bind(&ukf_slam::observe, objectIndex, msmt(1), transform, _1),
    covar.asDiagonal());
  //TODO: make sure this is a copy. Pretty sure it is
  return estimator.state;
}

VectorXd ukf_slam::updateDepth(double msmt, double covar)
{
  MatrixXd measurement(1, 1);
  measurement(0,0) = msmt;
  MatrixXd covariance(1, 1);
  covariance(0,0) = covar;
  estimator.predict(&propogate, 0.01 * MatrixXd::Identity(3*N, 3*N));	//Covar matrix is non zero at (2,2)
  estimator.correct(measurement,	//Vector1d probs doesn't work
   &ukf_slam::observe_depth,
    covariance);
  return estimator.state;
}


