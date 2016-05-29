// Piecewise polynomial trajectories
// by James A. Preiss

#include <string.h> // memset, memcpy

#include "pptraj.h"

// polynomials are stored with ascending degree

// evaluate a polynomial using horner's rule.
float polyval(float const *poly, int deg, float t)
{
    float x = 0.0;
    for (int i = deg; i >= 0; --i) {
        x = x * t + poly[i];
    }
    return x;
}

// compute derivative of a polynomial in place
void polyder(float *p, int deg)
{
	for (int i = 1; i <= deg; ++i) {
		p[i-1] = i * p[i];
	}
	p[deg] = 0;
}

void polyder4d(struct poly4d *p)
{
	for (int i = 0; i < 4; ++i) {
		polyder(p->p[i], PP_DEGREE);
	}
}

void poly4d_set_constant(struct poly4d *p, int dim, float x)
{
	p->p[dim][0] = x;
	for (int i = 1; i <= PP_DEGREE; ++i) {
		p->p[dim][i] = 0;
	}
}

struct vec polyval_xyz(struct poly4d const *p, float t)
{
	return mkvec(
		polyval(p->p[0], PP_DEGREE, t),
		polyval(p->p[1], PP_DEGREE, t),
		polyval(p->p[2], PP_DEGREE, t)
	);
}

float polyval_yaw(struct poly4d const *p, float t)
{
	return polyval(p->p[3], PP_DEGREE, t);
}

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
