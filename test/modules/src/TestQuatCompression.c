#include "unity.h"
#include "math3d.h"
#include "quatcompress.h"
#include "stdlib.h"

static int r()
{
	return rand() - ((int)RAND_MAX / 2);
}

void testQuatCodecRoundtrip() 
{
	for (int i = 0; i < 1000000; ++i) {
		float f[4];
		struct quat q = qnormalized(mkquat(r(), r(), r(), r()));
		qstoref(q, f);
		uint32_t comp = quatcompress(f);

		float ff[4];
		quatdecompress(comp, ff);
		struct quat qq = qloadf(ff);
		struct quat diff = qqmul(qinv(q), qq);

		// TODO get single rot angle instead of rpy
		struct vec diff_euler = quat2rpy(diff);
		TEST_ASSERT(fabs(diff_euler.x) < 0.01);
		TEST_ASSERT(fabs(diff_euler.y) < 0.01);
		TEST_ASSERT(fabs(diff_euler.z) < 0.01);
	}
}
