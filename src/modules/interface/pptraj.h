#pragma once

// Piecewise polynomial trajectories
// by James A. Preiss

#include "math3d.h"

#define PP_DEGREE (7)
#define PP_SIZE (PP_DEGREE + 1)
#define PP_MAX_PIECES (10)


//
// 1d polynomial functions.
//

// evaluate a polynomial using horner's rule.
float polyval(float const p[PP_SIZE], float t);

// construct a linear polynomial from p(0) = x0 to p(duration) = x1.
void polylinear(float p[PP_SIZE], float duration, float x0, float x1);

// plan a degree-5 polynomial with the given duration T,
// and given initial/final position, velocity, and acceleration
void poly5(float poly[PP_SIZE], float T,
	float x0, float dx0, float ddx0,
	float xf, float dxf, float ddxf);

// scale a polynomial in place.
void polyscale(float p[PP_SIZE], float s);

// compute the derivate of a polynomial in place.
void polyder(float p[PP_SIZE]);

// e.g. if s==2 the new polynomial will be stretched to take 2x longer
void polystretchtime(float p[PP_SIZE], float s);


//
// 4d single polynomial piece for x-y-z-yaw, includes duration.
//

struct poly4d
{
	float p[4][PP_SIZE];
	float duration; // TODO use int millis instead?
};

// construct a 4d zero polynomial.
struct poly4d poly4d_zero(float duration);

// construct a 4d linear polynomial.
struct poly4d poly4d_linear(float duration, 
	struct vec p0, struct vec p1, float yaw0, float yaw1);

// scale a 4d polynomial in-place.
void poly4d_scale(struct poly4d *p, float x, float y, float z, float yaw);

// shift a 4d polynomial by the given values in-place.
void poly4d_shift(struct poly4d *p, float x, float y, float z, float yaw);
static inline void poly4d_shift_vec(struct poly4d *p, struct vec pos, float yaw) {
	poly4d_shift(p, pos.x, pos.y, pos.z, yaw);
}

// e.g. if s==2 the new polynomial will be stretched to take 2x longer.
void poly4d_stretchtime(struct poly4d *p, float s);

// compute the derivative of a 4d polynomial in-place.
void polyder4d(struct poly4d *p);

// compute loose maximum of acceleration - 
// uses L1 norm instead of Euclidean, evaluates polynomial instead of root-finding
float poly4d_max_accel_approx(struct poly4d const *p);


// stored simple trajectories. could store only z to save memory.
extern struct piecewise_traj pp_figure8;

// output of differentially flat 4d polynomials.
struct traj_eval
{
	struct vec pos;
	struct vec vel;
	struct vec acc;
	struct vec omega;
	float yaw;
};

// a special value of traj_eval that indicates an invalid result.
struct traj_eval traj_eval_invalid(void);

// check if a traj_eval represents an invalid result.
bool is_traj_eval_valid(struct traj_eval const *ev);

// evaluate a single polynomial piece
struct traj_eval poly4d_eval(struct poly4d const *p, float t, float mass);



// ----------------------------------//
// piecewise polynomial trajectories //
// ----------------------------------//

struct piecewise_traj
{
	struct poly4d pieces[PP_MAX_PIECES];
	float t_begin;
	unsigned char n_pieces;
};

void piecewise_plan_5th_order(struct piecewise_traj *p, float duration,
	struct vec p0, float y0, struct vec v0, float dy0, struct vec a0,
	struct vec p1, float y1, struct vec v1, float dy1, struct vec a1);

void piecewise_plan_7th_order_no_jerk(struct piecewise_traj *p, float duration,
	struct vec p0, float y0, struct vec v0, float dy0, struct vec a0,
	struct vec p1, float y1, struct vec v1, float dy1, struct vec a1);

struct traj_eval piecewise_eval(
  struct piecewise_traj const *traj, float t, float mass);

void piecewise_scale(struct piecewise_traj *pp, float x, float y, float z, float yaw);

void piecewise_shift(struct piecewise_traj *pp, float x, float y, float z, float yaw);
static inline void piecewise_shift_vec(struct piecewise_traj *pp, struct vec pos, float yaw) { 
	piecewise_shift(pp, pos.x, pos.y, pos.z, yaw);
}

void piecewise_stretchtime(struct piecewise_traj *pp, float s);

static inline bool piecewise_is_finished(struct piecewise_traj const *traj, float t)
{
  float total_dur = 0;
  for (int i = 0; i < traj->n_pieces; ++i) {
    total_dur += traj->pieces[i].duration;
  }
  return (t - traj->t_begin) >= total_dur;
}

// TODO own file?
struct ellipse_traj
{
	struct vec center;
	struct vec major;
	struct vec minor;
	float period;
	float t_begin;
	//float phase;
};

struct traj_eval ellipse_traj_eval(struct ellipse_traj const *e, float t, float mass);

void plan_into_ellipse(struct traj_eval const *now,
                       struct ellipse_traj const *ellipse,
                       struct piecewise_traj *catchup,
                       float mass);
