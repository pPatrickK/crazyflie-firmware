#include "unity.h"
#include "num.h"
#include "pptraj.h"

static float const TOL = 0.0000001;

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
		pp.cursor = 0;
		pp.t_begin_piece = 0;
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

		pp.cursor = 1;
		pp.t_begin_piece = 0;
		ev = piecewise_eval(&pp, 0, 1);
		TEST_ASSERT_FLOAT_WITHIN(TOL, x1, ev.pos.x);
		TEST_ASSERT_FLOAT_WITHIN(TOL, 0, ev.pos.y);
		TEST_ASSERT_FLOAT_WITHIN(TOL, 0, ev.pos.z);
		TEST_ASSERT_FLOAT_WITHIN(TOL, 0, ev.yaw);

		ev = piecewise_eval(&pp, dur1, 1);
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
		TEST_ASSERT_FLOAT_WITHIN(TOL, ev.x, ev_stretch.x);
		TEST_ASSERT_FLOAT_WITHIN(TOL, ev.y, ev_stretch.y);
		TEST_ASSERT_FLOAT_WITHIN(TOL, ev.z, ev_stretch.z);
	}
}
