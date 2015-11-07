#ifndef UKF_H_
#define UKF_H_

#include <eigen3/Eigen/Dense>
// Can't use <functional> because target is C03
#include <boost/function.hpp>

using namespace Eigen;

/* This class implements an Unscented Kalman Filter. The main reference used
 * is Bayesian Estimation and Tracking: A Practical Guide by Anton J Haug. 
 */
class ukf
{
    public:
        ukf(VectorXd initialState, MatrixXd initialCovariance);
        
        /* The propogate parameter is a function which propogates a state vector
         * from time t to time t+1. The processNoise parameter is the covariance
         * matrix of the noise added from propogation.
         */
        void predict(boost::function<void (Ref<VectorXd>)> propogate,
            const MatrixXd processNoise);
        
        /* The observe parameter is a function which takes in a matrix of state
         * vectors and returns a matrix containing the corresponding measurement
         * vectors. The measurementNoise parameter is the covariance of the 
         * measurement parameter.
         */
        void correct(const VectorXd measurement, 
            boost::function<MatrixXd (MatrixXd)> observe,
            const MatrixXd measurementNoise);
            
        // Current state estimate
        VectorXd state;
        
        // Current covariance estimate
        MatrixXd covariance;
    
    private:
        MatrixXd generateSigmas();
        void recoverPrediction(Ref<MatrixXd>, const MatrixXd);
        void recoverCorrection(Ref<MatrixXd>, Ref<MatrixXd>,
            const VectorXd, const MatrixXd);
};

#endif 
