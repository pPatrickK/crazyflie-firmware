#include <Eigen/Dense>

extern "C" float testEigen(void)
{
	// Eigen::Vector3f a(1,2,3);
	// Eigen::Vector3f b(4,5,6);

	// Eigen::Vector3f c = a + b;

	// return c(0);

	// big Matrices do not fully work yet
	const int size = 3;
	Eigen::Matrix<float, size, size> m(Eigen::Matrix<float, size, size>::Random());
	// m(0,0) = 5;
	// m.setRandom();
	Eigen::Matrix<float, size, size> m2 = m.inverse();

	return m2(0, 0);
}
