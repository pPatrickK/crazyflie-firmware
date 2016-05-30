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
	float duration; // TODO use int millis instead?
};

struct traj_eval
{
	struct vec pos;
	struct vec vel;
	struct vec omega;
	float yaw;
};

// evaluate a single polynomial piece
struct traj_eval poly4d_eval(struct poly4d const *p, float t, float mass);


// stored simple trajectories. could store only z to save memory.
extern struct poly4d poly4d_takeoff;
extern struct poly4d poly4d_landing;

// useful for shifting the takeoff/land trajectories
void poly4d_shift(struct poly4d *p, float x, float y, float z, float yaw);


// ----------------------------------//
// piecewise polynomial trajectories //
// ----------------------------------//

struct piecewise_traj
{
	struct poly4d pieces[PP_MAX_PIECES];
	float t_begin_piece;
	unsigned char cursor;
	unsigned char n_pieces;
};

struct traj_eval piecewise_eval(struct piecewise_traj *traj, float t, float mass);
