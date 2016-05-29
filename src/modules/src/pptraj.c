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

// polynomials are stored with ascending degree

struct poly4d poly4d_takeoff = {
	.p = {{0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, },
	      {0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, },
	      {-0.000000, 0.000000, -0.000000, 2.128369, -3.133618, 2.164124, -0.768719, 0.109821, },
	      {0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, }},
	.duration = 2
};

struct poly4d poly4d_landing = {
	.p = {{0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, },
	      {0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, },
	      {1.000000, 0.000000, 0.000000, -2.022341, 2.977522, -2.056210, 0.730332, -0.104330, },
	      {0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, }},
	.duration = 2
};

void poly4d_shift(struct poly4d *p, float x, float y, float z, float yaw)
{
	p->p[0][0] += x;
	p->p[1][0] += y;
	p->p[2][0] += z;
	p->p[3][0] += yaw;
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
	struct vec acc = polyval_xyz(&deriv, t);

	// 3rd derivative
	polyder4d(&deriv);
	struct vec jerk = polyval_xyz(&deriv, t);

	struct vec thrust = vadd(acc, mkvec(0, 0, GRAV));
	float thrust_mag = mass * vmag(thrust);

	struct vec z_body = vdiv(thrust, thrust_mag);
	struct vec x_world = mkvec(cos(out.yaw), sin(out.yaw), 0);
	struct vec y_body = vnormalized(vcross(z_body, x_world));
	struct vec x_body = vcross(y_body, z_body);
	
	struct vec jerk_orth_zbody = vorthunit(jerk, z_body);
	struct vec h_w = vscl(mass / thrust_mag, jerk_orth_zbody);

	out.omega.x = -vdot(h_w, y_body);
	out.omega.y = vdot(h_w, x_body);
	out.omega.z = z_body.z * dyaw;

	return out;
}
