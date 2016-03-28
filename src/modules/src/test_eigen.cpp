#include <Eigen/Dense>

extern "C" float testEigen(void)
{
	Eigen::Vector3f a(1,2,3);
	Eigen::Vector3f b(4,5,6);

	Eigen::Vector3f c = a + b;

	return c(0);

	// Matrices do not fully work yet
	// Eigen::Matrix<float, 16, 16> m(Eigen::Matrix<float, 16, 16>::Random());
	// // m(0,0) = 5;
	// // m.setRandom();
	// Eigen::Matrix<float, 16, 16> m2 = m.inverse();

	// return m2(0, 0);
}
