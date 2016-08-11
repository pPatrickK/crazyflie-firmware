/**
 *    ______
 *   / ____/________ _____  __  ________      ______ __________ ___
 *  / /   / ___/ __ `/_  / / / / / ___/ | /| / / __ `/ ___/ __ `__ \
 * / /___/ /  / /_/ / / /_/ /_/ (__  )| |/ |/ / /_/ / /  / / / / / /
 * \____/_/   \__,_/ /___/\__, /____/ |__/|__/\__,_/_/  /_/ /_/ /_/
 *                       /____/
 *
 * Crazyswarm advanced control firmware for Crazyflie
 *
 * Copyright (C) 2016 Wolfgang Hoenig and James Preiss,
 * University of Southern California
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * pptraj.c: implementation of xyz-yaw piecewise polynomial trajectories
 *           see Mellinger and Kumar, "Minimum Snap...", ICRA 2011
 */

#include "pptraj.h"
#include "mathconstants.h"

// polynomials are stored with ascending degree

struct poly4d poly4d_takeoff = {
	.p = {{0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, },
	      {0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, },
	      {-0.000000, 0.000000, -0.000000, 2.128369, -3.133618, 2.164124, -0.768719, 0.109821, },
	      {0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, }},
	.duration = 2
};

static void polyscale(float p[PP_SIZE], float s)
{
	for (int i = 0; i < PP_SIZE; ++i) {
		p[i] *= s;
	}
}

// e.g. if s==2 the new polynomial will be stretched to take 2x longer
static void polystretchtime(float p[PP_SIZE], float s)
{
	float recip = 1.0f / s;
	float scale = recip;
	for (int i = 1; i < PP_SIZE; ++i) {
		p[i] *= scale;
		scale *= recip;
	}
}

void poly4d_scale(struct poly4d *p, float x, float y, float z, float yaw)
{
	polyscale(p->p[0], x);
	polyscale(p->p[1], y);
	polyscale(p->p[2], z);
	polyscale(p->p[3], yaw);
}

void poly4d_shift(struct poly4d *p, float x, float y, float z, float yaw)
{
	p->p[0][0] += x;
	p->p[1][0] += y;
	p->p[2][0] += z;
	p->p[3][0] += yaw;
}

void poly4d_stretchtime(struct poly4d *p, float s)
{
	for (int i = 0; i < 4; ++i) {
		polystretchtime(p->p[i], s);
	}
	p->duration *= s;
}

// evaluate a polynomial using horner's rule.
static float polyval(float const *poly, int deg, float t)
{
    float x = 0.0;
    for (int i = deg; i >= 0; --i) {
        x = x * t + poly[i];
    }
    return x;
}

// compute derivative of a polynomial in place
static void polyder(float *p, int deg)
{
	for (int i = 1; i <= deg; ++i) {
		p[i-1] = i * p[i];
	}
	p[deg] = 0;
}

static void polyder4d(struct poly4d *p)
{
	for (int i = 0; i < 4; ++i) {
		polyder(p->p[i], PP_DEGREE);
	}
}

static struct vec polyval_xyz(struct poly4d const *p, float t)
{
	return mkvec(
		polyval(p->p[0], PP_DEGREE, t),
		polyval(p->p[1], PP_DEGREE, t),
		polyval(p->p[2], PP_DEGREE, t)
	);
}

static float polyval_yaw(struct poly4d const *p, float t)
{
	return polyval(p->p[3], PP_DEGREE, t);
}

// compute loose maximum of acceleration - 
// uses L1 norm instead of Euclidean, evaluates polynomial instead of root-finding
float poly4d_max_accel_approx(struct poly4d const *p)
{
	static struct poly4d acc;
	acc = *p;
	polyder4d(&acc);
	polyder4d(&acc);
	int steps = 10 * p->duration;
	int step = p->duration / (steps - 1);
	float t = 0;
	float amax = 0;
	for (int i = 0; i < steps; ++i) {
		struct vec ddx = polyval_xyz(&acc, t);
		float ddx_minkowski = vminkowski(ddx);
		if (ddx_minkowski  > amax) amax = ddx_minkowski;
		t += step;
	}
	return amax;
}

struct traj_eval traj_eval_invalid()
{
	struct traj_eval ev;
	ev.pos = vrepeat(NAN);
	return ev;
}

bool is_traj_eval_valid(struct traj_eval const *ev)
{
	return visnan(ev->pos);
}

struct traj_eval poly4d_eval(struct poly4d const *p, float t, float mass)
{
	// flat variables
	struct traj_eval out;
	out.pos = polyval_xyz(p, t);
	out.yaw = polyval_yaw(p, t);

	// 1st derivative
	static struct poly4d deriv;
	deriv = *p;
	polyder4d(&deriv);
	out.vel = polyval_xyz(&deriv, t);
	float dyaw = polyval_yaw(&deriv, t);

	// 2nd derivative
	polyder4d(&deriv);
	out.acc = polyval_xyz(&deriv, t);

	// 3rd derivative
	polyder4d(&deriv);
	struct vec jerk = polyval_xyz(&deriv, t);

	struct vec thrust = vadd(out.acc, mkvec(0, 0, GRAV));
	float thrust_mag = mass * vmag(thrust);

	struct vec z_body = vnormalize(thrust);
	struct vec x_world = mkvec(cos(out.yaw), sin(out.yaw), 0);
	struct vec y_body = vnormalize(vcross(z_body, x_world));
	struct vec x_body = vcross(y_body, z_body);

	struct vec jerk_orth_zbody = vorthunit(jerk, z_body);
	struct vec h_w = vscl(mass / thrust_mag, jerk_orth_zbody);

	out.omega.x = -vdot(h_w, y_body);
	out.omega.y = vdot(h_w, x_body);
	out.omega.z = z_body.z * dyaw;

	return out;
}

// piecewise eval
struct traj_eval piecewise_eval(
  struct piecewise_traj const *traj, float t, float mass)
{
	int cursor = 0;
	t = t - traj->t_begin;
	while (cursor < traj->n_pieces) {
		struct poly4d const *piece = &(traj->pieces[cursor]);
		if (t <= piece->duration) {
			return poly4d_eval(piece, t, mass);
		}
		t -= piece->duration;
		++cursor;
	}
	// if we get here, the trajectory has ended
	struct poly4d const *end_piece = &(traj->pieces[traj->n_pieces - 1]);
	struct traj_eval ev = poly4d_eval(end_piece, end_piece->duration, mass);
	ev.vel = vzero();
	ev.acc = vzero();
	ev.omega = vzero();
	return ev;
}

void piecewise_shift(struct piecewise_traj *pp, float x, float y, float z, float yaw)
{
	for (int i = 0; i < PP_MAX_PIECES; ++i) {
		poly4d_shift(&pp->pieces[i], x, y, z, yaw);
	}
}

void piecewise_scale(struct piecewise_traj *pp, float x, float y, float z, float yaw)
{
	for (int i = 0; i < PP_MAX_PIECES; ++i) {
		poly4d_scale(&pp->pieces[i], x, y, z, yaw);
	}
}

void piecewise_stretchtime(struct piecewise_traj *pp, float s)
{
	for (int i = 0; i < PP_MAX_PIECES; ++i) {
		poly4d_stretchtime(&pp->pieces[i], s);
	}
}

static void poly5(float poly[PP_SIZE], float T,
	float x0, float dx0, float ddx0,
	float xf, float dxf, float ddxf)
{
	float T2 = T * T;
	float T3 = T2 * T;
	float T4 = T3 * T;
	float T5 = T4 * T;
	poly[0] = x0;
	poly[1] = dx0;
	poly[2] = ddx0 / 2;
	poly[3] = (-12*dx0*T - 8*dxf*T - 3*ddx0*T2 + ddxf*T2 - 20*x0 + 20*xf)/(2*T3);
	poly[4] = (16*dx0*T + 14*dxf*T + 3*ddx0*T2 - 2*ddxf*T2 + 30*x0 - 30*xf)/(2*T4);
	poly[5] = (-6*dx0*T - 6*dxf*T - ddx0*T2 + ddxf*T2 - 12*x0 + 12*xf)/(2*T5);
	for (int i = 6; i < PP_SIZE; ++i) {
		poly[i] = 0;
	}
};

static void poly7_nojerk(float poly[PP_SIZE], float T,
	float x0, float dx0, float ddx0,
	float xf, float dxf, float ddxf)
{
	float T2 = T * T;
	float T3 = T2 * T;
	float T4 = T3 * T;
	float T5 = T4 * T;
	float T6 = T5 * T;
	float T7 = T6 * T;
	poly[0] = x0;
	poly[1] = dx0;
	poly[2] = ddx0/2;
	poly[3] = 0;
	poly[4] = -(5*(14*x0 - 14*xf + 8*T*dx0 + 6*T*dxf + 2*T2*ddx0 - T2*ddxf))/(2*T4);
	poly[5] = (84*x0 - 84*xf + 45*T*dx0 + 39*T*dxf + 10*T2*ddx0 - 7*T2*ddxf)/T5;
	poly[6] = -(140*x0 - 140*xf + 72*T*dx0 + 68*T*dxf + 15*T2*ddx0 - 13*T2*ddxf)/(2*T6);
	poly[7] = (2*(10*x0 - 10*xf + 5*T*dx0 + 5*T*dxf + T2*ddx0 - T2*ddxf))/T7;
	for (int i = 8; i < PP_SIZE; ++i) {
		poly[i] = 0;
	}
}

// y, dy == yaw, derivative of yaw
void piecewise_plan_5th_order(struct piecewise_traj *pp, float duration,
	struct vec p0, float y0, struct vec v0, float dy0, struct vec a0,
	struct vec p1, float y1, struct vec v1, float dy1, struct vec a1)
{
	pp->n_pieces = 1;
	struct poly4d *p = &pp->pieces[0];
	p->duration = duration;
	poly5(p->p[0], duration, p0.x, v0.x, a0.x, p1.x, v1.x, a1.x);
	poly5(p->p[1], duration, p0.y, v0.y, a0.y, p1.y, v1.y, a1.y);
	poly5(p->p[2], duration, p0.z, v0.z, a0.z, p1.z, v1.z, a1.z);
	poly5(p->p[3], duration, y0, dy0, 0, y1, dy1, 0);
}

// y, dy == yaw, derivative of yaw
void piecewise_plan_7th_order_no_jerk(struct piecewise_traj *pp, float duration,
	struct vec p0, float y0, struct vec v0, float dy0, struct vec a0,
	struct vec p1, float y1, struct vec v1, float dy1, struct vec a1)
{
	pp->n_pieces = 1;
	struct poly4d *p = &pp->pieces[0];
	p->duration = duration;
	poly7_nojerk(p->p[0], duration, p0.x, v0.x, a0.x, p1.x, v1.x, a1.x);
	poly7_nojerk(p->p[1], duration, p0.y, v0.y, a0.y, p1.y, v1.y, a1.y);
	poly7_nojerk(p->p[2], duration, p0.z, v0.z, a0.z, p1.z, v1.z, a1.z);
	poly7_nojerk(p->p[3], duration, y0, dy0, 0, y1, dy1, 0);
}

static struct vec aubv(float a, struct vec u, float b, struct vec v)
{
	return mkvec(a*u.x + b*v.x, a*u.y + b*v.y, a*u.z + b*v.z);
}

struct traj_eval ellipse_traj_eval(struct ellipse_traj const *e, float t, float mass)
{
	struct traj_eval out;

	float s = 2 * M_PI / e->period;
	float s2 = s * s;
	float s3 = s2 * s;

	float cos_t = cos(s * t);
	float sin_t = sin(s * t);

	out.pos    = vadd(aubv( cos_t,    e->major,     sin_t, e->minor), e->center);
	out.vel         = aubv(-s*sin_t,  e->major,   s*cos_t, e->minor);
	out.acc         = aubv(-s2*cos_t, e->major, -s2*sin_t, e->minor);
	struct vec jerk = aubv( s3*sin_t, e->major, -s3*cos_t, e->minor);

	out.yaw = 0; // TODO: care about yaw?
	float dyaw = 0;

	// TODO: factor out common code with poly trajs.
	struct vec thrust = vadd(out.acc, mkvec(0, 0, GRAV));
	float thrust_mag = mass * vmag(thrust);

	struct vec z_body = vnormalize(thrust);
	struct vec x_world = mkvec(cos(out.yaw), sin(out.yaw), 0);
	struct vec y_body = vnormalize(vcross(z_body, x_world));
	struct vec x_body = vcross(y_body, z_body);

	struct vec jerk_orth_zbody = vorthunit(jerk, z_body);
	struct vec h_w = vscl(mass / thrust_mag, jerk_orth_zbody);

	out.omega.x = -vdot(h_w, y_body);
	out.omega.y = vdot(h_w, x_body);
	out.omega.z = z_body.z * dyaw;

	return out;
}

void plan_into_ellipse(struct traj_eval const *now,
                       struct ellipse_traj const *ellipse,
                       struct piecewise_traj *catchup,
                       float mass)
{
  // crazyflie peak thrust is approx 60 grams, mass is approx 30 grams -
  // should have enough to accelerate at approx gravity rate.
  // halve it to be conservative
  float MAX_ACCEL = GRAV / 2;

  float t_catchup = ellipse->period / 8;
  float amax;
  int iter = 1;
  do {
    struct traj_eval ev_ell = 
      ellipse_traj_eval(ellipse, t_catchup, mass);
    piecewise_plan_5th_order(catchup, t_catchup,
      now->pos,   now->yaw,   now->vel,   now->omega.z,   now->acc,
      ev_ell.pos, ev_ell.yaw, ev_ell.vel, ev_ell.omega.z, ev_ell.acc);
    amax = poly4d_max_accel_approx(&catchup->pieces[0]);

    // this should rarely happen in practice, but it helps pass unit tests
    // with randomized vectors :)
    float a_ends = fmax(vminkowski(now->acc), vminkowski(ev_ell.acc));
    if (a_ends > MAX_ACCEL) {
      MAX_ACCEL = a_ends * 1.1;
    }

    t_catchup *= 2;
    ++iter;
  } while (iter <= 4 && amax > MAX_ACCEL);
}

