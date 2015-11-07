#include "ukf_pose.h"
#include "ukf_slam.h"
#include "ukf.h"
#include "gtest/gtest.h"
#include <eigen3/Eigen/Dense>
#include <iostream>

using namespace Eigen;

// Convenience assertion for Eigen types
void expect_matrix_near(MatrixXd expected, MatrixXd actual, double tol) {
  bool isCorrect = (expected - actual).isMuchSmallerThan(1.0, tol);
  EXPECT_TRUE(isCorrect) << "Expected:\n" << expected << "\nActual:\n" << actual << "\n";
}

// Tests that gtest is working.
TEST(sanity, one_equals_one) {
  EXPECT_EQ(1,1);
}

TEST(eigen_normalized, no_aliasing) {
  Vector3d v, v_expected;
  v_expected << 0, 0.6, 0.8;
  v << 0, 3, 4;
  v = v_expected;
  EXPECT_EQ(v, v_expected);
}

TEST(eigen_angle_axis, basic) {
  Vector3d v, v_expected;
  v << 0, 3, 4;
  AngleAxisd a(v.norm(), v.normalized());
  EXPECT_DOUBLE_EQ(a.angle(), 5);
}

TEST(eigen_scaleVector, times_zero_is_zero) {
  Vector3d v;
  v << 1,2,3;
  v *= 0;
  for (int i = 0; i<3; i++)
  {
    EXPECT_DOUBLE_EQ(v(i), 0.0);
  }
}

TEST(eigen_scaleVector, inverse) {
  Vector3d v;
  v << 1.0,2,3;
  v = -1.0 * v;
  for (int i = 0; i<3; i++)
  {
    EXPECT_DOUBLE_EQ(v(i),(double) -(i+1));
  }
}

TEST(eigen_cholesky, diagonal_matrix)
{
  Matrix3d matrix;
  matrix << 4,0,0,0,4,0,0,0,4;
  Matrix3d result;

  result = matrix.llt().matrixU();

  for (int i = 0; i < 9; i++)
  {
    if (i%4)
    {
      EXPECT_DOUBLE_EQ(result(i), 0.0);
    }else
    {
      EXPECT_DOUBLE_EQ(result(i), 2.0);
    }
  }
}

TEST(eigen_cholesky, wiki_matrix)
{
  Matrix3d matrix, result, expected;
  matrix << 4,12,-16,12,37,-43,-16,-43,98;
  expected << 2,0,0,6,1,0,-8,5,3;

  result = matrix.llt().matrixL();


  EXPECT_EQ(result, expected);
  /*for (int i = 0; i < 9; i++)
  {
      EXPECT_DOUBLE_EQ(result[i], expected[i]);
  }*/
}

TEST(matrix_utils_add, simple_add)
{
  Vector3d v1, v2;
  v1 << 1,2,3;
  v2 << 5,4,3;

  v1 += v2;

  for(int i = 0; i < 3; i++)
  {
    EXPECT_DOUBLE_EQ(v1[i], 6.0);
    EXPECT_DOUBLE_EQ(v2[i], 5 - i);
  }
}

TEST(eigen_rowwise_mean, basic) {
  Matrix3d m;
  VectorXd result;
  m << 1, 2, 2, 2, 1, 2, 1, 2, 2;
  result = m.rowwise().mean();
  for (int i = 0; i < 3; i++) {
    EXPECT_DOUBLE_EQ(result(i), 5./3.);
  }
}

TEST(eigen_stream, concat_vectors) {
  Matrix<double, 2, 4> m;
  Vector2d v1(0,1),v2(2,3),v3(4,5),v4(6,7);
  m << v1, v2, v3, v4;

  for (int i = 0; i < 4; i++) {
    EXPECT_DOUBLE_EQ(m(0, i), 2.0*i);
    EXPECT_DOUBLE_EQ(m(1, i), 2.0*i + 1);
  }
}

TEST(eigen_averageOuterProduct, simple_outerProduct)
{
  VectorXd v1(5), v2(3);
  MatrixXd result(5, 3), expectedResult(5, 3);
  v1 << 1,2,3,4,5;
  v2 << 1,2,3;
  expectedResult << 1,2,3,2,4,6,3,6,9,4,8,12,5,10,15;

  result = v1*v2.transpose();

  for(int i = 0; i < 5; i++)
  {
    EXPECT_DOUBLE_EQ(v1(i), i + 1);
  }

  for(int i = 0; i < 3; i++)
  {
    EXPECT_DOUBLE_EQ(v2(i), i + 1);
  }

  EXPECT_EQ(expectedResult, result);
}

TEST(eigen_subtractMultiple, simple_subtract)
{
  MatrixXd m(5, 2);
  VectorXd v(5), result(5);
  m << 1,1,2,2,3,3,4,4,5,5;
  v << 1,2,3,4,5;

  m.colwise() -= v;

  for(int i = 0; i < 10; i++)
  {
    EXPECT_DOUBLE_EQ(0.0, m(i));
  }

  for(int i = 0; i < 5; i++)
  {
    EXPECT_DOUBLE_EQ(i%5+1, v(i));
  }
}

TEST(eigen_solve, simple_solve)
{
  MatrixXd A(5,3), B(3,3), C(5,3);
  A << 
    6.894238064697299,   7.101870550729457,   8.669369896175127,
    19.686538668181953,  19.516795180339010,  18.794131786875756,
    32.478839271666608 , 31.931719809948561,  28.918893677576385,
    45.271139875151263,  44.346644439558119,  39.043655568277018,
    58.063440478635911,  56.761569069167663,  49.168417458977643;

  B <<
     2.039447372786358,   1.819167793214639,   0.405485035160554,
     1.819167793214639 ,  1.674718492450819,   0.644421924204393,
     0.405485035160554,   0.644421924204393,   2.325013670868595;

  C = B.transpose().ldlt().solve(A.transpose()).transpose();
  //C = A*B.inverse();

  for (int i = 0; i<15; i++)
  {
    EXPECT_NEAR(i+1, C(i/3, i%3), 1e-10);
  }
}

TEST(eigen_multiply, simple_multiply)
{
  MatrixXd A(2,3), B(3,4), C(2,4), expectedC(2,4);
  A << 1,2,3,4,5,6;
  B << 4,0,12,12,8,20,32,3,11,2,0,9;
  C << 1,2,3,4,5,6,7,8;

  expectedC << 54,  48,  79,  49, 127, 118, 215, 125;

  C += A*B;

  for(int i = 0; i < 8; i++)
  {
    EXPECT_DOUBLE_EQ(expectedC(i), C(i));
  }
}

TEST(eigen_rotateThisByThat,aRotation)
{
  Vector3d v, rotation, result, expected;
  v << 10,0,0;
  rotation << 1,1,1;
  expected << 2.262956,9.567123,-1.830079;

  result = AngleAxisd(rotation.norm(), rotation.normalized()).toRotationMatrix() * v;

  expect_matrix_near(expected, result, 1e-6);
}

TEST(eigen_rotateThisByThat,multipleRotations)
{
  double pi2 = 3.14159265359/2.0;
  Vector3d x, x90, y90, z90, v1, v2;
  x << 1,0,0;
  x90 << pi2,0,0;
  y90 << 0,pi2,0;
  z90 << 0,0,pi2;
  v1 << 1,0,0;
  v2 = AngleAxisd(z90.norm(), z90.normalized()).toRotationMatrix() * v1;
  v1 = AngleAxisd(x90.norm(), x90.normalized()).toRotationMatrix() * v2;
  v2 = AngleAxisd(y90.norm(), y90.normalized()).toRotationMatrix() * v1;

  expect_matrix_near(x, v2, 1e-10);
}

TEST(matrix_utils_composeRotations, simple_composition)
{
  Vector3d A, B, C, expectedC;
  A << 0.98327,0.928374,0.12635;
  B << 0.423876,0.324682,0.938745;
  expectedC << 0.88729352, 1.65648765, 1.00913355;

  AngleAxisd composition(AngleAxisd(B.norm(), B.normalized())*AngleAxisd(A.norm(), A.normalized()));
  
  C = composition.angle()*composition.axis();

  expect_matrix_near(expectedC, C, 1e-8);
}

TEST(pose_ukf_observe, zero_state)
{
  Vector3d state, expected;
  MatrixXd result;
  state << 0,0,0;
  expected << 0,0,9.8;

  result = ukf_pose::observe(state.replicate(1,6));

  expect_matrix_near(expected.replicate(1,6), result, 1e-10);
}

TEST(pose_ukf_observe, z_rotation)
{
  Vector3d state, expected;
  MatrixXd result;
  state << 0,0,1.139847;
  expected << 0,0,9.8;

  result = ukf_pose::observe(state.replicate(1,6));
  
  expect_matrix_near(expected.replicate(1,6), result, 1e-10);
}

TEST(pose_ukf_observe, x_rotation)
{
  double pi2 = 3.14159265359/2.0;
  Vector3d state, expected;
  MatrixXd result;
  state << pi2,0,0;
  expected << 0,9.8,0;

  result = ukf_pose::observe(state.replicate(1,6));

  expect_matrix_near(expected.replicate(1,6), result, 1e-10);
  
}

TEST(pose_ukf_observe, y_rotation)
{
  double pi4 = 3.14159265359/4.0;
  double sqrt2 = 1.41421356237;
  Vector3d state, expected;
  MatrixXd result;
  state << 0,pi4,0;
  expected << -9.8/sqrt2, 0, 9.8/sqrt2;

  result = ukf_pose::observe(state.replicate(1,6));

  expect_matrix_near(expected.replicate(1,6), result, 1e-10);
}

TEST(pose_ukf_propogate, x_rotation)
{
  Vector3d state, rotation, expected;
  state << 0.1,0,0;
  rotation << 0.1,0,0;
  expected << 0.2,0,0;

  ukf_pose::propogate(rotation, state);
  expect_matrix_near(expected, state, 1e-10);
}

TEST(pose_ukf_propogate, y_rotation)
{
  Vector3d state, rotation, expected;
  state << 0, 0.1,0;
  rotation << 0, 0.1, 0;
  expected << 0, 0.2, 0;

  ukf_pose::propogate(rotation, state);
  expect_matrix_near(expected, state, 1e-10);
}

TEST(pose_ukf_propogate, z_rotation)
{
  Vector3d state, rotation, expected;
  state << 0,0,0.1;
  rotation << 0,0,0.1;
  expected << 0,0,0.2;

  ukf_pose::propogate(rotation, state);
  expect_matrix_near(expected, state, 1e-10);
}

void propogate(Ref<Eigen::VectorXd> state)
{
  state *= 2.;
}



TEST(ukf_predict, 2x) {
  Vector3d initialState, expectedState;
  Matrix3d initialCovar, expectedCovar;
  initialState << 2, 3, 1;
  expectedState << 4, 6, 2;
  initialCovar << 1,  2,  1,
                   2, 13, 14,
                   1, 14, 42;
  expectedCovar << 4,  8,  4,
                   8, 52, 56,
                   4, 56, 168;
  ukf estimator(initialState, initialCovar);
  estimator.predict(&propogate, Matrix3d::Zero());

  expect_matrix_near(expectedState, estimator.state, 1e-10);
  expect_matrix_near(expectedCovar, estimator.covariance, 1e-10);
}


MatrixXd observe(MatrixXd sigma)
{
  return -sigma;
}

TEST(ukf_correct, minusx) {
  Vector3d initialState, msmt, expectedState;
  Matrix3d initialCovar, expectedCovar;
  initialState << 2, 3, 1;
  msmt << -3, -3, -1;
  expectedState << 2.5, 3, 1;
  initialCovar << 1,  2,  1,
                  2, 13, 14,
                  1, 14, 42;
  ukf estimator(initialState, initialCovar);
  
  estimator.correct(msmt, &observe, initialCovar);

  expect_matrix_near(expectedState, estimator.state, 1e-10);
  expect_matrix_near(0.5*initialCovar, estimator.covariance, 1e-10);
}

TEST(ukf_slam_update, basic) {
  ukf_slam slam(1);
  Vector2d result;
  slam.update(Vector2d::Zero(), result, 0);
  expect_matrix_near(Vector2d::Zero(), result, 1e-10);
}

TEST(ukf_slam_update, repeat) {
  ukf_slam slam(1);
  Vector2d result;
  for (int i = 0; i < 10000; i++) {
    slam.update(Vector2d::Zero(), result, 0);
  }
  expect_matrix_near(Vector2d::Zero(), result, 1e-10);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
