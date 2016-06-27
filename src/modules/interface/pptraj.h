#pragma once

// Piecewise polynomial trajectories
// by James A. Preiss

#include "math3d.h"

#define PP_DEGREE (7)
#define PP_SIZE (PP_DEGREE + 1)
#define PP_MAX_PIECES (10)

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

static inline struct poly4d poly4d_zero(float duration)
{
	struct poly4d p;
	p.duration = duration;
	memset(p.p, 0, sizeof(p.p));
	return p;
}

static inline struct poly4d poly4d_linear(float duration, struct vec p0, struct vec p1, float yaw0, float yaw1)
{
	struct poly4d p = poly4d_zero(duration);
	struct vec slope_p = vdiv(vsub(p1, p0), duration);
	float slope_yaw = (yaw1 - yaw0) / duration;
	p.p[0][0] = p0.x;
	p.p[1][0] = p0.y;
	p.p[2][0] = p0.z;
	p.p[3][0] = yaw0;
	p.p[0][1] = slope_p.x;
	p.p[1][1] = slope_p.y;
	p.p[2][1] = slope_p.z;
	p.p[3][1] = slope_yaw;
	return p;
}

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
void poly4d_scale(struct poly4d *p, float x, float y, float z, float yaw);
// e.g. if s==2 the new polynomial will be stretched to take 2x longer
void poly4d_stretchtime(struct poly4d *p, float s);


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

void piecewise_shift(struct piecewise_traj *pp, float x, float y, float z, float yaw);
void piecewise_scale(struct piecewise_traj *pp, float x, float y, float z, float yaw);
void piecewise_stretchtime(struct piecewise_traj *pp, float s);

struct traj_eval piecewise_eval(struct piecewise_traj *traj, float t, float mass);

static inline bool piecewise_is_finished(struct piecewise_traj const *traj)
{
	return traj->cursor == traj->n_pieces;
}
