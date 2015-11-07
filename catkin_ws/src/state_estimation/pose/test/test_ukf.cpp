#include "matrix_utils.h"
#include "rotation_vector_utils.h"
#include "ukf.h"
#include "gtest/gtest.h"


// Tests that gtest is working.
TEST(sanity, one_equals_one) {
  EXPECT_EQ(1,1);
}

TEST(matrix_utils_scaleVector, times_zero_is_zero) {
	double vector[] = {1,2,3,4,5};
	scaleVector(0.0, vector, 5);
	for (int i = 0; i<5; i++)
	{
		EXPECT_DOUBLE_EQ(vector[i], 0.0);
	}
}

TEST(matrix_utils_scaleVector, inverse) {
	double vector[] = {1.0,2,3,4,5};
	scaleVector(-1.0, vector, 5);
	for (int i = 0; i<5; i++)
	{
		EXPECT_DOUBLE_EQ(vector[i],(double) -(i+1));
	}
}

TEST(matrix_utils_cholesky, diagonal_matrix)
{
	double matrix[] = {4,0,0,0,4,0,0,0,4};
	double result[9];

	cholesky(matrix, result, 3);

	for (int i = 0; i < 9; i++)
	{
		if (i%4)
		{
			EXPECT_DOUBLE_EQ(result[i], 0.0);
		}else
		{
			EXPECT_DOUBLE_EQ(result[i], 2.0);
		}
	}
}

TEST(matrix_utils_cholesky, wiki_matrix)
{
	double matrix[] = {4,12,-16,12,37,-43,-16,-43,98};
	double result[9];
	double expected[] = {2,0,0,6,1,0,-8,5,3};

	cholesky(matrix, result, 3);

	for (int i = 0; i < 9; i++)
	{
		EXPECT_DOUBLE_EQ(result[i], expected[i]);
	}
}

TEST(matrix_utils_copy, simple_copy)
{
	double vector1[] = {1,2,3,4,5};
	double vector2[10];

	vectorCopy(vector1, vector2, 5);
	vectorCopy(vector1, &(vector2[5]), 5);

	for(int i = 0; i < 5; i++)
	{
		EXPECT_DOUBLE_EQ(vector1[i%5], vector2[i]);
	}
}

TEST(matrix_utils_add, simple_add)
{
	double v1[] = {1,2,3,4,5};
	double v2[] = {5,4,3,2,1};

	addVectors(v1,v2,5);

	for(int i = 0; i < 5; i++)
	{
		EXPECT_DOUBLE_EQ(v1[i], 6.0);
		EXPECT_DOUBLE_EQ(v2[i], 5 - i);
	}
}

TEST(matrix_utils_sub, simple_sub)
{
	double v1[] = {1,2,3,4,5};
	double v2[] = {1,2,3,4,5};

	subtractVectors(v1,v2,5);

	for(int i = 0; i < 5; i++)
	{
		EXPECT_DOUBLE_EQ(v1[i], 0.0);
		EXPECT_DOUBLE_EQ(v2[i], i + 1);
	}
}

TEST(matrix_utils_vectorIndex, simple_index)
{
	double test[3];
	EXPECT_EQ(&(test[2]), vectorIndex(test,2,1));
}


TEST(matrix_utils_averageVectors, simple_average)
{
	double v1[] = {1,2,3,4,5,1,2,3,4,5};
	double expectedResult[] = {1,2,3,4,5};
	double result[5];

	averageVectors(v1,result,2,5);

	for(int i = 0; i < 10; i++)
	{
		EXPECT_DOUBLE_EQ(i%5 + 1, v1[i]);
	}

	for(int i = 0; i < 5; i++)
	{
		EXPECT_DOUBLE_EQ(expectedResult[i], result[i]);
	}
}

TEST(matrix_utils_averageOuterProduct, simple_outerProduct)
{
	double v1[] = {1,2,3,4,5,1,2,3,4,5};
	double v2[] = {1,2,3,1,2,3};
	double result[15] = {};
	double expectedResult[] = {1,2,3,2,4,6,3,6,9,4,8,12,5,10,15};

	averageOuterProduct(v1,v2,result,2,5,3);

	for(int i = 0; i < 5; i++)
	{
		EXPECT_DOUBLE_EQ(v1[i], i + 1);
	}

	for(int i = 0; i < 3; i++)
	{
		EXPECT_DOUBLE_EQ(v2[i], i + 1);
	}

	for(int i = 0; i < 15; i++)
	{
		EXPECT_DOUBLE_EQ(expectedResult[i], result[i]);
	}
}

TEST(matrix_utils_subtractMultiple, simple_subtract)
{
	double v1[] = {1,2,3,4,5,1,2,3,4,5};
	double v2[] = {1,2,3,4,5};
	double result[5];

	subtractMultipleVectors(v1,v2,2,5);

	for(int i = 0; i < 10; i++)
	{
		EXPECT_DOUBLE_EQ(0.0, v1[i]);
	}

	for(int i = 0; i < 5; i++)
	{
		EXPECT_DOUBLE_EQ(i%5+1, v2[i]);
	}
}

TEST(matrix_utils_solve, simple_solve)
{

	double A[] =
	 {6.894238064697299,   7.101870550729457,   8.669369896175127,
	  19.686538668181953,  19.516795180339010,  18.794131786875756,
	  32.478839271666608 , 31.931719809948561,  28.918893677576385,
	  45.271139875151263,  44.346644439558119,  39.043655568277018,
	  58.063440478635911,  56.761569069167663,  49.168417458977643};

	double B[] =
	   {2.039447372786358,   1.819167793214639,   0.405485035160554,
	   1.819167793214639 ,  1.674718492450819,   0.644421924204393,
	   0.405485035160554,   0.644421924204393,   2.325013670868595};

	double C[15] = {};

	solve(A,B,C,5,3);

	for (int i = 0; i<15; i++)
	{
		 EXPECT_NEAR(i+1, C[i], 1e-10);
	}
}

TEST(matrix_utils_multiply, simple_multiply)
{
	double A[] = {1,2,3,4,5,6};
	double B[] = {4,0,12,12,8,20,32,3,11,2,0,9};
	double C[] = {1,2,3,4,5,6,7,8};

	double expectedC[] = {54,  48,  79,  49, 127, 118, 215, 125};

	leftMultiplyAdd(A,B,C, 2,3,4);

	for(int i = 0; i < 8; i++)
	{
		EXPECT_DOUBLE_EQ(expectedC[i], C[i]);
	}
}

TEST(matrix_utils_transposeMultiply, simple_multiply)
{
	double A[] = {1,2,3,4,5,6};
	double B[] = {4,8,11,0,20,2,12,32,0,12,3,9};
	double C[] = {1,2,3,4,5,6,7,8};

	double expectedC[] = {54,  48,  79,  49, 127, 118, 215, 125};

	transposedMultiplyAdd(A,B,C, 2,3,4);

	for(int i = 0; i < 8; i++)
	{
		EXPECT_DOUBLE_EQ(expectedC[i], C[i]);
	}
}

TEST(rotation_utils_rotateThisByThat,aRotation)
{
	double vector[] = {10,0,0};
	double rotation[] = {1,1,1};
	double result[3];
	double expected[] = {2.262956,9.567123,-1.830079};

	rotateThisByThat(vector, rotation, result);

	for(int i = 0; i < 3; i++)
	{
		EXPECT_NEAR(expected[i], result[i], 1e-6);
	}
}

TEST(rotation_utils_rotateThisByThat,multipleRotations)
{
    double pi2 = 3.14159265359/2.0;
	double x[] = {1,0,0};
    double x90[] = {pi2,0,0};
    double y90[] = {0,pi2,0};
    double z90[] = {0,0,pi2};
    double v1[] = {1,0,0};
    double v2[3];
    rotateThisByThat(v1, z90, v2);
    rotateThisByThat(v2, x90,v1);
    rotateThisByThat(v1, y90, v2);

    for(int i = 0; i < 3; i++)
    {
    	EXPECT_NEAR(x[i], v2[i], 1e-10);
    }
}

TEST(matrix_utils_composeRotations, simple_composition)
{
	double A[] = {0.98327,0.928374,0.12635};
	double B[] = {0.423876,0.324682,0.938745};
	double C[3] = {};
	double expectedC[] = {0.88729352, 1.65648765, 1.00913355};
	composeRotations(A,B,C);

	for (int i = 0; i< 3; i++)
	{
		EXPECT_NEAR(expectedC[i], C[i], 1e-8);
	}
}

TEST(pose_ukf_h, zero_state)
{
	double state[] = {0,0,0,0,0,0,0,0,0};
	double result[3];
	double expected[] = {0,0,9.8};

	h(state,result);

	for (int i = 0; i < 3; i++)
	{
		EXPECT_NEAR(expected[i], result[i], 1e-10);
	}
}

TEST(pose_ukf_h, z_rotation)
{
	double state[] = {0,0,1.139847,0,0,0,0,0,0};
	double result[3];
	double expected[] = {0,0,9.8};

	h(state,result);

	for (int i = 0; i < 3; i++)
	{
		EXPECT_NEAR(expected[i], result[i], 1e-10);
	}
}

TEST(pose_ukf_h, x_rotation)
{
	double pi2 = 3.14159265359/2.0;
	double state[] = {pi2,0,0,0,0,0,0,0,0};
	double result[3];
	double expected[] = {0,9.8,0};

	h(state,result);

	for (int i = 0; i < 3; i++)
	{
		EXPECT_NEAR(expected[i], result[i], 1e-10);
	}
}

TEST(pose_ukf_h, y_rotation)
{
	double pi4 = 3.14159265359/4.0;
	double sqrt2 = 1.41421356237;
	double state[] = {0,pi4,0,0,0,0,0,0,0};
	double result[3];
	double expected[] = {-9.8/sqrt2, 0, 9.8/sqrt2};

	h(state,result);

	for (int i = 0; i < 3; i++)
	{
		EXPECT_NEAR(expected[i], result[i], 1e-10);
	}
}

void expect_array_near(double* expected, double* actual, int length, double tolerance)
{
	for (int i = 0; i < length; i++)
	{
		EXPECT_NEAR(expected[i], actual[i], tolerance);
	}
}

TEST(pose_ukf_propogate, x_rotation)
{
	double state[] = {0.1,0,0};
	double rotation[] = {0.1,0,0};
	double expected[] = {0.2,0,0};

	propogate(rotation, state);
	expect_array_near(expected, state, 3, 1e-10);
}

TEST(pose_ukf_propogate, y_rotation)
{
	double state[] = {0, 0.1,0};
	double rotation[] = {0, 0.1, 0};
	double expected[] = {0, 0.2, 0};

	propogate(rotation, state);
	expect_array_near(expected, state, 3, 1e-10);
}

TEST(pose_ukf_propogate, z_rotation)
{
	double state[] = {0,0,0.1};
	double rotation[] = {0,0,0.1};
	double expected[] = {0,0,0.2};

	propogate(rotation, state);
	expect_array_near(expected, state, 3, 1e-10);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
