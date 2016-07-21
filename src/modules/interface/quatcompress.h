#include <stdint.h>
#include <math.h>

// 
// Compress and decompress a quaternion into a 32-bit value.
// by James A. Preiss, 2016.
// MIT License.
//

// assumes input quaternion is normalized. will fail if not.
static inline uint32_t quatcompress(float const q[4])
{
	// we send the values of the quaternion's smallest 3 elements.
	unsigned i_largest = 0;
	for (unsigned i = 1; i < 4; ++i) {
		if (fabs(q[i]) > fabs(q[i_largest])) {
			i_largest = i;
		}
	}

	// since -q represents the same rotation as q,
	// transform the quaternion so the largest element is positive.
	// this avoids having to send its sign bit.
	unsigned negate = q[i_largest] < 0;

	// 1/sqrt(2) is the largest possible value 
	// of the second-largest element in a unit quaternion.
	float SMALL_MAX = 0.70710678118;

	// do compression using sign bit and 9-bit precision per element.
	uint32_t comp = i_largest;
	for (unsigned i = 0; i < 4; ++i) {
		if (i != i_largest) {
			unsigned negbit = (q[i] < 0) ^ negate;
			// TODO rounding?
			unsigned mag = ((1 << 9) - 1) * (fabs(q[i]) / SMALL_MAX);
			comp = (comp << 10) | (negbit << 9) | mag;
		}
	}

	return comp;
}

static inline void quatdecompress(uint32_t comp, float q[4])
{
	float const SMALL_MAX = 0.70710678118;
	unsigned const mask = (1 << 9) - 1;

	int const i_largest = comp >> 30;
	float sum_squares = 0;
	for (int i = 3; i >= 0; --i) {
		if (i != i_largest) {
			unsigned mag = comp & mask;
			unsigned negbit = (comp >> 9) & 0x1;
			comp = comp >> 10;
			q[i] = SMALL_MAX * ((float)mag) / mask;
			if (negbit == 1) {
				q[i] = -q[i];
			}
			sum_squares += q[i] * q[i];
		}
	}

	q[i_largest] = sqrtf(1.0f - sum_squares);
}
