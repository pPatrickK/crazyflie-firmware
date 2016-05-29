#pragma once

// Piecewise polynomial trajectories
// by James A. Preiss

#include "math3d.h"

#define PP_DEGREE (7)
#define PP_SIZE (PP_DEGREE + 1)
#define PP_MAX_PIECES (8)

// 4d single polynomial piece for x-y-z-yaw.
//
// if we ever run on x86-SSE or arm-NEON, 
// transposing the coeff array and using 4xfloat SIMD math would be a big speedup
//
struct poly4d
{
	float p[4][PP_SIZE];
	float duration;
};

void poly4d_set_constant(struct poly4d *p, int dim, float x);

struct traj_eval
{
	struct vec pos;
	struct vec vel;
	struct vec omega;
	float yaw;
};
struct traj_eval poly4d_eval(struct poly4d const *p, float t, float mass);

// stored simple trajectories. could store only z to save memory.
extern struct poly4d poly4d_takeoff;
extern struct poly4d poly4d_landing;
