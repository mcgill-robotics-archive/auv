#include "ukf.h"
#include <math.h>
#include <stdio.h>

ukf::ukf(VectorXd initialState, MatrixXd initialCovariance) :
  state(initialState),
  covariance(initialCovariance)
{}

/* This method generates 2*state.size() states distributed on a hypersphere
 * around state and returns them as a size x 2*size matrix.
 */
MatrixXd ukf::generateSigmas()
{
  /* The sigmas matrix is recreated at each update step because the size of
   * the state vector may have changed.
   */
  MatrixXd sigmas(state.size(), 2*state.size());
  MatrixXd sqrtC = covariance.llt().matrixL();
  sigmas << -sqrt(state.size()) * sqrtC, sqrt(state.size()) * sqrtC;
  sigmas.colwise() += state;
  return sigmas;
}

void ukf::recoverPrediction(Ref<MatrixXd> sigmas,
    const MatrixXd processNoise)
{
  state = sigmas.rowwise().mean();
  sigmas.colwise() -= state;
  covariance = sigmas*sigmas.transpose()/(2*(double) state.size())
      + processNoise;
}

void ukf::predict(boost::function<void (Ref<VectorXd>)> propogate,
    const MatrixXd processNoise)
{
  MatrixXd sigmas = generateSigmas();
  for (int i = 0; i < sigmas.cols(); i++)
  {
    propogate(sigmas.col(i));
  }
  recoverPrediction(sigmas, processNoise);
}

void ukf::correct(const VectorXd measurement, boost::function<MatrixXd (MatrixXd)> observe,
    const MatrixXd measurementNoise)
{
  MatrixXd sigmas = generateSigmas();
  /* It would be nice to have the observe function be called with a vector and
   * return a vector. The problem then would be how do we know what size of
   * matrix to allocate to hold the returned vectors. We could use a template
   * to tell us, but one of the possible values for a matrix size is dynamic,
   * meaning that to figure out the size of the returned vectors we would have 
   * to call observe and then allocate a matrix. That way seems hacky, so the
   * current solution will do for now.
   */
  MatrixXd gammas = observe(sigmas);
  recoverCorrection(sigmas, gammas, measurement, measurementNoise);
}

void ukf::recoverCorrection(Ref<MatrixXd> sigmas, Ref<MatrixXd> gammas,
    const VectorXd measurement, const MatrixXd measurementNoise)
{
  VectorXd predMsmt = gammas.rowwise().mean();

  sigmas.colwise() -= state;
  gammas.colwise() -= predMsmt;
  MatrixXd measCovar = gammas * gammas.transpose()/(2. * state.size()) 
      + measurementNoise;
  MatrixXd crossCovar = sigmas * gammas.transpose()/(2. * state.size());

  MatrixXd gain = measCovar.transpose().ldlt()
                      .solve(crossCovar.transpose()).transpose();

  state += gain * (measurement - predMsmt);
  covariance -= crossCovar * gain.transpose();
}
