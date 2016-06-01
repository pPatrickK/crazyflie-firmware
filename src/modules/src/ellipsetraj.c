#include <math.h>
#include "math3d.h"

// TEMP HACK - factor out common trajectory stuff?
// or mush everything together into e.g. analytictraj.c?
#include "pptraj.h"

struct ellipse_traj
{
	struct vec major;
	struct vec minor;
	float phase;
	float time_scale;
};

struct vec aubv(float a, struct vec u, float b, struct vec v)
{
	return mkvec(a*u.x + b*v.x, a*u.y + b*v.y, a*u.z + b*v.z);
}

struct traj_eval ellipse_traj_eval(struct ellipse_traj const *e, float t, float mass)
{
	struct traj_eval out;

	float s = e->time_scale;
	float s2 = s * s;
	float s3 = s2 * s;

	float cos_t = cos(s * t);
	float sin_t = sin(s * t);

	out.pos  = aubv( cos_t, e->major,  sin(t), e->minor);
	out.vel  = aubv(-s*sin_t, e->major,  s*cos(t), e->minor);
	struct vec acc  = aubv(-s2*cos_t, e->major, -s2*sin(t), e->minor);
	struct vec jerk = aubv( s3*sin_t, e->major, -s3*cos(t), e->minor);

	out.yaw = 0; // TODO: care about yaw?
	float dyaw = 0;

	// TODO: factor out common code with poly trajs.
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
