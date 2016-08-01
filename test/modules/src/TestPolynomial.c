#include "unity.h"
#include "num.h"
#include "pptraj.h"
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

void testTimestretchPiecewiseLinear() 
{
	float x0 = 1, x1 = -1, x2 = 3;
	float dur0 = 0.5, dur1 = 1;

	struct piecewise_traj pp;
	pp.pieces[0] = poly4d_linear(dur0, mkvec(x0, 0, 0), mkvec(x1, 0, 0), 0, 0);
	pp.pieces[1] = poly4d_linear(dur1, mkvec(x1, 0, 0), mkvec(x2, 0, 0), 0, 0);
	pp.n_pieces = 2;
	
	for (int i = 0; i < 2; ++i) {
		// sanity check - unstretched
		pp.t_begin = 0;
		struct traj_eval ev = piecewise_eval(&pp, 0, 1);
		TEST_ASSERT_FLOAT_WITHIN(TOL, x0, ev.pos.x);
		TEST_ASSERT_FLOAT_WITHIN(TOL, 0, ev.pos.y);
		TEST_ASSERT_FLOAT_WITHIN(TOL, 0, ev.pos.z);
		TEST_ASSERT_FLOAT_WITHIN(TOL, 0, ev.yaw);

		ev = piecewise_eval(&pp, dur0, 1);
		TEST_ASSERT_FLOAT_WITHIN(TOL, x1, ev.pos.x);
		TEST_ASSERT_FLOAT_WITHIN(TOL, 0, ev.pos.y);
		TEST_ASSERT_FLOAT_WITHIN(TOL, 0, ev.pos.z);
		TEST_ASSERT_FLOAT_WITHIN(TOL, 0, ev.yaw);

		ev = piecewise_eval(&pp, dur0 + dur1, 1);
		TEST_ASSERT_FLOAT_WITHIN(TOL, x2, ev.pos.x);
		TEST_ASSERT_FLOAT_WITHIN(TOL, 0, ev.pos.y);
		TEST_ASSERT_FLOAT_WITHIN(TOL, 0, ev.pos.z);
		TEST_ASSERT_FLOAT_WITHIN(TOL, 0, ev.yaw);

		// do it for 2nd loop
		piecewise_stretchtime(&pp, 2.0);
		dur0 *= 2;
		dur1 *= 2;
	}
}

void testTimestretchTakeoff()
{
	struct poly4d takeoff_stretched = poly4d_takeoff;
	poly4d_stretchtime(&takeoff_stretched, 2.0);
	TEST_ASSERT_FLOAT_WITHIN(TOL, takeoff_stretched.duration, poly4d_takeoff.duration * 2.0);
	for (float t = 0; t <= poly4d_takeoff.duration; t += 0.125) {
		struct vec ev = poly4d_eval(&poly4d_takeoff, t, 1).pos;
		struct vec ev_stretch = poly4d_eval(&takeoff_stretched, 2*t, 1).pos;
		TEST_ASSERT_VEC_WITHIN(TOL, ev, ev_stretch);
	}
}

void testPlan5thOrder()
{
	struct piecewise_traj pp;
	srand(100); // deterministic
	int TRIALS = 10000;
	for (int i = 0; i < TRIALS; ++i) {
		struct traj_eval p0 = random_traj_point();
		struct traj_eval p1 = random_traj_point();
		// since random traj points can be quite far apart,
		// need to use only fairly long durations
		float duration = randu(1, 5);

		piecewise_plan_5th_order(&pp, duration,
			p0.pos, p0.yaw, p0.vel, p0.omega.z, p0.acc,
			p1.pos, p1.yaw, p1.vel, p1.omega.z, p1.acc);
		pp.t_begin = 0;

		struct traj_eval t0 = piecewise_eval(&pp, 0, MASS);
		struct traj_eval t1 = piecewise_eval(&pp, duration, MASS);
		if (!traj_close(p0, t0)) {
			print_traj_pt("p0", p0);
			print_traj_pt("t0", t0);
			TEST_ASSERT(traj_close(p0, t0));
		}
		if (!traj_close(p1, t1)) {
			print_traj_pt("p1", p1);
			print_traj_pt("t1", t1);
			TEST_ASSERT(traj_close(p1, t1));
		}
	}
}

// TODO work on 7th-order planning - right now errors are kind of large

void testPlanIntoEllipse()
{
	srand(100); // deterministic
	int TRIALS = 10000;
	for (int i = 0; i < TRIALS; ++i) {
		struct ellipse_traj ellipse;
		ellipse.center = vecrandu(-5, 5);
		ellipse.major = vecrandu(-5, 5);
		ellipse.minor = vecrandu(-5, 5);
		ellipse.period = 20; // TODO shorten if ellipse isn't too big
		ellipse.t_begin = 0;

		struct traj_eval now = random_traj_point();
		struct piecewise_traj pp;

		plan_into_ellipse(&now, &ellipse, &pp, MASS);
		pp.t_begin = 0;
		float duration = pp.pieces[0].duration;
		//printf("dur guess: %f, dur: %f\n", ellipse.period / 8, duration);

		struct traj_eval t0 = piecewise_eval(&pp, 0, MASS);
		if (!traj_close(now, t0)) {
			print_traj_pt("now", now);
			print_traj_pt("t0", t0);
			TEST_ASSERT(traj_close(now, t0));
		}

		struct traj_eval ellipse_join = 
			ellipse_traj_eval(&ellipse, duration, MASS);
		struct traj_eval t1 = piecewise_eval(&pp, duration, MASS);
		if (!traj_close(ellipse_join, t1)) {
			print_traj_pt("ellipse_join", ellipse_join);
			print_traj_pt("t1", t1);
			TEST_ASSERT(traj_close(ellipse_join, t1));
		}
	}
}
