#include "unity.h"
#include "math3d.h"
#include "quatcompress.h"
#include "stdlib.h"

static int r()
{
	return rand() - ((int)RAND_MAX / 2);
}

static void printquat(struct quat q)
{
	printf("[%.10f, %.10f, %.10f, %.10f]\n", q.x, q.y, q.z, q.w);
}

static float abserr(struct quat q)
{
	float f[4];
	qstoref(q, f);
	uint32_t comp = quatcompress(f);
	float ff[4];
	quatdecompress(comp, ff);
	struct quat qq = qloadf(ff);
	struct quat diff = qnormalize(qqmul(qinv(q), qq));
	float angle = quatangle(diff);
	if (angle > M_PI) {
		angle = angle - 2 * M_PI;
	}
	return fabs(angle);
}

void testQuatCodecFuzz() 
{
	double angle_sum = 0.0;
	double worst_angle = 0.0;
	int const N = 1000000;
	for (int i = 0; i < N; ++i) {
		struct quat q = qnormalized(mkquat(r(), r(), r(), r()));
		float angle = abserr(q);
		angle_sum += angle;
		if (angle > worst_angle) {
			worst_angle = angle;
		}
	}
	TEST_ASSERT(worst_angle < radians(0.27));
	printf("mean error: %f degrees\n", degrees(angle_sum / N));
	printf("worst error: %f degrees\n", degrees(worst_angle));
}

void testQuatCodecFuzzTiny() 
{
	double angle_sum = 0.0;
	double worst_angle = 0.0;
	int const N = 1000000;
	for (int i = 0; i < N; ++i) {
		struct quat q = qnormalized(mkquat(r()/1000.0, r()/1000.0, r()/1000.0, r()));
		float angle = abserr(q);
		angle_sum += angle;
		if (angle > worst_angle) {
			worst_angle = angle;
		}
	}
	TEST_ASSERT(worst_angle < radians(0.27));
	printf("mean error: %f degrees\n", degrees(angle_sum / N));
	printf("worst error: %f degrees\n", degrees(worst_angle));
}

void testQuatCodecSpecial()
{
	// max size of 2nd-largest
	float const SMALL_MAX = 1.0 / sqrt(2);
	float errLargestEqual = abserr(mkquat(0, 0, SMALL_MAX, SMALL_MAX));
	TEST_ASSERT(errLargestEqual < radians(0.2));
}
