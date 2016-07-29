#include "ekf.h"
#include "mathconstants.h"
#include "unity.h"

#include <float.h>

bool close(float a, float b, float tol) { return fabs(a - b) < tol; }
bool vclose(struct vec a, struct vec b, float tol)
{
	return close(a.x, b.x, tol) && close(a.y, b.y, tol) && close(a.z, b.z, tol);
}
bool qclose(struct quat a, struct quat b, float tol)
{
	return close(a.x, b.x, tol) && close(a.y, b.y, tol) && 
	       close(a.z, b.z, tol) && close(a.w, b.w, tol);
}

void testQuat()
{
	// math tests
	{	// zero gyro -> no rotation
		struct quat q = qeye();
		struct quat q1 = quat_gyro_update(q, vzero(), 0.1);
		TEST_ASSERT(qclose(q, q1, FLT_MIN));

		q = qaxisangle(mkvec(100,-2,3), 0.15);
		struct quat qi = qinv(q);
		struct quat qiq = qqmul(qi, q);
		TEST_ASSERT(qclose(qiq, qeye(), 0.000001));
	}

	{ // full circle pitch - the very fine dt is needed for exactness
		struct quat q = qeye();
		int N = 100000;
		struct vec rpy = mkvec(0, 2 * M_PI / N, 0);
		for (int i = 0; i < N; ++i) {
			q = quat_gyro_update(q, rpy, 1.0 / N);
		}
		TEST_ASSERT(qclose(q, qeye(), 0.0001)); // loose tolerance for dumb euler integration
	}

	{
		struct vec x = mkvec(1, 0, 0);
		struct vec y = mkvec(0, 1, 0);
		struct vec z = mkvec(0, 0, 1);
		struct vec rx;
		rx = qvrot(qaxisangle(z, radians(90)), x);
		TEST_ASSERT(vclose(rx, y, 0.000001));
		rx = qvrot(qaxisangle(z, radians(180)), x);
		TEST_ASSERT(vclose(rx, vneg(x), 0.000001));
		rx = qvrot(qaxisangle(z, radians(90)), y);
		TEST_ASSERT(vclose(rx, vneg(x), 0.000001));

		struct vec v_arbitrary = mkvec(1, 2, -1.2);
		struct quat q_arbitrary = qaxisangle(v_arbitrary, radians(36));
		rx = x;
		for (int i = 0; i < 10; ++i) {
			rx = qvrot(q_arbitrary, rx);
		}
		TEST_ASSERT(vclose(rx, x, 0.000001));
	}
}

void testEKF()
{
	// system tests
	struct ekf a, b;
	struct ekf *front = &a, *back = &b;
	//back->acc_var = 0.0;
	//back->gyro_var = 0.0;
	//back->bias_acc_var = 0.0;
	//back->bias_gyro_var = 0.0;

	// acceleration only
	float zeros[4] = {0, 0, 0, 0};
	struct quat q = qeye();
	ekf_init(back,  zeros, zeros, (float *)&q);
	ekf_init(front, zeros, zeros, (float *)&q);
	float dt = 1.0 / 100.0;
	float acc[3] = {0, 0, 1 + GRAV};
	float gyro[3] = {0, 0, 0};
	for (int i = 0; i < 100; ++i) {
		ekf_imu(back, front, acc, gyro, dt);
		struct ekf *tmp = front; front = back; back = tmp; // swap
	}
	TEST_ASSERT(vclose(back->vel, mkvec(0, 0, 1), 0.000001));

	float vicon_pos[] = {0, 0, 0};
	float vicon_quat[] = {0, 0, 0, 1};
	ekf_vicon(back, front, vicon_pos, vicon_quat);
}
