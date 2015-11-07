#include "ukf_pose.h"
#include "ukf.h"
#include <math.h>
#include <stdio.h>
#include <iostream>
#include <boost/bind.hpp>

const double INITIAL_COVARIANCE = 0.0000001;
const double PROCESS_VARIANCE = 0.00000004;
const double MEASUREMENT_VARIANCE = 0.025;

ukf_pose::ukf_pose() :
  estimator(VectorXd::Zero(3), INITIAL_COVARIANCE * MatrixXd::Identity(3, 3)),
  processNoise(PROCESS_VARIANCE * MatrixXd::Identity(3,3)),
  measurementNoise(MEASUREMENT_VARIANCE * MatrixXd::Identity(3,3))
{}

AngleAxisd ukf_pose::angleAxis(Vector3d v) {
  if (v.norm() > 0) {
    return AngleAxisd(v.norm(), v.normalized());
  } else {
    return AngleAxisd(0., Vector3d::UnitZ());
  }
}

void ukf_pose::propogate(const Vector3d rotation, Ref<Vector3d> state) {
  AngleAxisd resultTransform(angleAxis(state) * angleAxis(rotation));  

  //We want to choose the rotation vector closest to sigma
  double angle = resultTransform.angle() + 2 * M_PI * 
      floor(0.5 + (state.dot(resultTransform.axis())-resultTransform.angle())
      /(2 * M_PI));

  state = angle * resultTransform.axis();
}

//Return gammas, take in full sigmas, cycle through cols in here
MatrixXd ukf_pose::observe(MatrixXd sigmas)
{
  Vector3d gravity(0,0,9.8);
  MatrixXd gammas(3, 6);
  for(int i = 0; i < sigmas.cols(); i++)
  {
    AngleAxisd transform = angleAxis(-sigmas.col(i));

    //Apply the transform to gravity
    gammas.col(i) = transform.toRotationMatrix() * gravity;
  }
  return gammas;
}

void ukf_pose::update(const Vector3d acc, const Vector3d rotation, 
    Ref<Vector3d> outPose)
{
  fixState(estimator.state);
  estimator.predict(boost::bind(&ukf_pose::propogate, rotation, _1),
      processNoise);
  estimator.correct(acc, &observe, measurementNoise);
  outPose = estimator.state;
}

void ukf_pose::fixState(Ref<Vector3d> state)
{
  double angle = state.norm();
  if (angle > M_PI)
  {
    state = (angle - 2*M_PI)*state.normalized();
  }
}
  


