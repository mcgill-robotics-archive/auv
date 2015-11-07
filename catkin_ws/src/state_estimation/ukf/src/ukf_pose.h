#ifndef UKF_POSE_
#define UKF_POSE_

#include <eigen3/Eigen/Dense>
#include "ukf.h"

using namespace Eigen;

class ukf_pose
{
  public:
    ukf_pose();
    /* Call this with the current acceleration and gyro*dt and the current state
     * will be written to outPose
     */
    void update(const Vector3d acc, const Vector3d rotation,
        Ref<Vector3d> outPose);
    static void propogate(const Vector3d, Ref<Vector3d>);
    static MatrixXd observe(MatrixXd);
    void fixState(Ref<Vector3d>);

  private:
    ukf estimator;
    MatrixXd processNoise;
    MatrixXd measurementNoise;
    static AngleAxisd angleAxis(Vector3d);
};

#endif
