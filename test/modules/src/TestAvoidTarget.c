#include "unity.h"
#include "num.h"

#include "avoidtarget.h"
#include "pptraj.h"
#include "fig8traj.c" // hack - how to link in?
#include "planner.h"
#include <stdlib.h>

//
// Utilities
//

static float const MASS = 0.03;

static inline float rand01()
{
	return rand() / ((float)RAND_MAX);
}

static inline float randu(float min, float max)
{
	return min + rand01() * (max - min);
}

static inline struct vec vecrandu(float min, float max)
{
	return mkvec(randu(min, max), randu(min, max), randu(min, max));
}

static struct traj_eval random_traj_point()
{
	struct traj_eval ev;
	ev.pos = vecrandu(-10, 10);
	ev.vel = vecrandu(-5, 5);
	ev.acc = vecrandu(-2, 2);
	ev.omega = vecrandu(-10, 10);
	ev.yaw = randu(-M_PI, M_PI);
	return ev;
}

static bool traj_close(struct traj_eval a, struct traj_eval b)
{
	return vleq(vabs(vsub(a.pos, b.pos)), vrepeat(0.001))
	    && vleq(vabs(vsub(a.vel, b.vel)), vrepeat(0.001))
	    && vleq(vabs(vsub(a.acc, b.acc)), vrepeat(0.001))
	    && fabs(a.yaw - b.yaw) < 0.001;
}

void print_traj_pt(char const *name, struct traj_eval ev)
{
	printf(
	"%s:\n"
	"\tpos = %f, %f, %f\n"
	"\tvel = %f, %f, %f\n"
	"\tacc = %f, %f, %f\n"
	"\tomega = %f, %f, %f\n"
	"\tyaw = %f\n\n",
	name,
	ev.pos.x, ev.pos.y, ev.pos.z,
	ev.vel.x, ev.vel.y, ev.vel.z,
	ev.acc.x, ev.acc.y, ev.acc.z,
	ev.omega.x, ev.omega.y, ev.omega.z,
	ev.yaw);
}

static float const TOL = 0.0000001;

#define TEST_ASSERT_VEC_WITHIN(tol, a, b) \
	TEST_ASSERT_FLOAT_WITHIN(tol, a.x, b.x); \
	TEST_ASSERT_FLOAT_WITHIN(tol, a.y, b.y); \
	TEST_ASSERT_FLOAT_WITHIN(tol, a.z, b.z);

void testAvoidTarget()
{
	struct avoid_target a;
	struct vec home = vecrandu(-1, 1);
	float const MAX_SPEED = 2.0;
	float const MAX_DISP = 1.0;

	// initial goal should be hover at home
	init_avoid_target(&a, home, MAX_SPEED, MAX_DISP, 0);
	struct traj_eval ev = eval_avoid_target(&a, 1.0f);
	TEST_ASSERT_VEC_WITHIN(TOL, ev.pos, home);
	TEST_ASSERT_VEC_WITHIN(TOL, ev.vel, vzero());
	TEST_ASSERT_VEC_WITHIN(TOL, ev.omega, vzero());

	// if target is very far away, goal should be close to hover at home
	init_avoid_target(&a, home, MAX_SPEED, MAX_DISP, 0);
	struct vec target = vrepeat(1e9);
	update_avoid_target(&a, target, 1.0f);
	ev = eval_avoid_target(&a, 1.0f);
	TEST_ASSERT_VEC_WITHIN(0.001, ev.pos, home);
	TEST_ASSERT_VEC_WITHIN(0.001, ev.vel, vzero());
	TEST_ASSERT_VEC_WITHIN(0.001, ev.omega, vzero());

	// same, but via planner
	struct planner p;
	plan_init(&p, MASS);
	plan_start_avoid_target(&p, home, MAX_DISP, MAX_SPEED, 0);
	plan_update_avoid_target(&p, target, 1.0f);
	ev = plan_current_goal(&p, 1.0f);
	TEST_ASSERT_VEC_WITHIN(0.001, ev.pos, home);
	TEST_ASSERT_VEC_WITHIN(0.001, ev.vel, vzero());
	TEST_ASSERT_VEC_WITHIN(0.001, ev.omega, vzero());

	// test that displacement is in the right direction
	home = vzero();
	target = mkvec(1, 0, 0);
	init_avoid_target(&a, home, MAX_SPEED, MAX_DISP, 0);
	update_avoid_target(&a, target, 0.0f);
	ev = eval_avoid_target(&a, 1.0f);
	TEST_ASSERT(ev.pos.x < -0.001);
	TEST_ASSERT_FLOAT_WITHIN(TOL, ev.pos.y, home.y);
	TEST_ASSERT_FLOAT_WITHIN(TOL, ev.pos.z, home.z);
}
