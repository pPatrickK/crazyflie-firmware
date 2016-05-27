#pragma once

// Piecewise polynomial trajectories
// by James A. Preiss

#include <string.h> // memset, memcpy

#include "ekfutil.h"


#define PP_DEGREE (7)
#define PP_SIZE (PP_DEGREE + 1)

// polynomials are stored with ascending degree

// evaluate a polynomial using horner's rule.
// TODO: convert to iterative form, GCC does not do tail call optimization on this
float polyval(float const *poly, int deg, float t)
{
	if (deg == 0) return poly[0];
	return poly[0] + t * polyval(poly+1, deg-1, t);
}

// compute derivative of a polynomial
void polyder(float const *p, int deg, float *dp)
{
	for (int i = 1; i <= deg; ++i) {
		dp[i-1] = i * p[i];
	}
}

// this struct contains the polynomial and all its derivatives, to avoid recomputing derivatives
// (however, computing derivatives on-the-fly might be worth the memory savings... measure if needed)
struct pp
{
	// 4 derivatives, 4 dimensions, degree
	float p[4][4][PP_SIZE];
	float t_end;
	float mass;
};

void pp_init(struct pp *pp, float xyzyaw[4][PP_SIZE], float t_end)
{
	pp->t_end = t_end;
	memset(pp->p, 0, sizeof(pp->p));
	memcpy(pp->p[0], xyzyaw, sizeof(pp->p[0]));
	// compute the derivatives
	for (int deriv = 1; deriv < 4; ++deriv) {
		for (int dim = 0; dim < 4; ++dim) {
			polyder(pp->p[deriv-1][dim], PP_DEGREE, pp->p[deriv][dim]);			
		}
	}
}

struct vec polyval3(float const p[3][PP_SIZE], float t)
{
	struct vec v;
	for (int i = 0; i < 3; ++i) {
		v.a[i] = polyval(p[i], PP_DEGREE, t);
	}
	return v;
}

struct traj_eval
{
	struct vec pos;
	struct vec vel;
	struct vec omega;
	float yaw;
};

struct traj_eval pp_eval(struct pp const *pp, float t)
{
	struct traj_eval out;
	out.pos = polyval3(pp->p[0], t);
	out.vel = polyval3(pp->p[1], t);
	out.yaw = polyval(pp->p[0][3], PP_DEGREE, t);
	
	struct vec acc = polyval3(pp->p[2], t);
	struct vec jerk = polyval3(pp->p[3], t);

	float dyaw = polyval(pp->p[1][3], PP_DEGREE - 1, t);

	struct vec thrust = vadd(acc, mkvec(0, 0, GRAV));
	float thrust_mag = pp->mass * vmag(thrust);

	struct vec z_body = vdiv(thrust, thrust_mag);
	struct vec x_world = mkvec(cos(out.yaw), sin(out.yaw), 0);
	struct vec y_body = vnormalized(vcross(z_body, x_world));
	struct vec x_body = vcross(y_body, z_body);
	
	struct vec jerk_orth_zbody = vorthunit(jerk, z_body);
	struct vec h_w = vscl(pp->mass / thrust_mag, jerk_orth_zbody);

	out.omega.x = -vdot(h_w, y_body);
	out.omega.y = vdot(h_w, x_body);
	out.omega.z = z_body.z * dyaw;

	return out;
}
