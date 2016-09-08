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
 * planner.c: trajectory planner state machine
 *
 */

#include "planner.h"


// ------------------------ //
// static helper functions. //
// ------------------------ //

static void plan_pp_flip(struct planner *p, float t)
{
	p->ppBack->t_begin = t;
	struct piecewise_traj* tmp = p->ppFront;
	p->ppFront = p->ppBack;
	p->ppBack = tmp;
	// this is necessary because we might recieve the startTrajectory command
	// repeatedly in a short time... TODO avoid doing this
	*p->ppBack = *p->ppFront;
}

static void plan_takeoff_or_landing(struct planner *p, struct vec pos, float yaw, float height, float duration)
{
	struct vec takeoff_pos = pos;
	takeoff_pos.z = height;

	piecewise_plan_7th_order_no_jerk(p->ppBack, duration,
		pos,         yaw, vzero(), 0, vzero(),
		takeoff_pos,   0, vzero(), 0, vzero());
}


// ----------------- //
// public functions. //
// ----------------- //

void plan_init(struct planner *p, float mass)
{
	p->mass = mass;
	p->ppFront = &p->pp1;
	p->ppBack = &p->pp2;
	p->state = TRAJECTORY_STATE_IDLE;
}

void plan_emergency_stop(struct planner *p)
{
	p->state = TRAJECTORY_STATE_IDLE;
}

bool plan_is_stopped(struct planner *p)
{
	return p->state == TRAJECTORY_STATE_IDLE;
}

bool plan_is_flying(struct planner *p)
{
	return 		p->state != TRAJECTORY_STATE_IDLE
			&& 	p->state != TRAJECTORY_STATE_TAKING_OFF
			&&  p->state != TRAJECTORY_STATE_LANDING;
}

struct traj_eval plan_current_goal(struct planner *p, float t)
{
	switch (p->state) {
		case TRAJECTORY_STATE_ELLIPSE_CATCHUP:
			if (piecewise_is_finished(p->ppFront, t)) {
				p->state = TRAJECTORY_STATE_ELLIPSE;
			}
			// it's OK, we'll eval the ellipse next time
			return piecewise_eval(p->ppFront, t, p->mass);

		case TRAJECTORY_STATE_ELLIPSE:
			t = t - p->ellipse.t_begin;
			return ellipse_traj_eval(&p->ellipse, t, p->mass);

		case TRAJECTORY_STATE_AVOID_TARGET:
			return eval_avoid_target(&p->avoid, t);

		case TRAJECTORY_STATE_TAKING_OFF:
			if (piecewise_is_finished(p->ppFront, t)) {
				p->state = TRAJECTORY_STATE_FLYING;
			}
			return piecewise_eval(p->ppFront, t, p->mass);

		case TRAJECTORY_STATE_LANDING:
			if (piecewise_is_finished(p->ppFront, t)) {
				p->state = TRAJECTORY_STATE_IDLE;
			}
			return piecewise_eval(p->ppFront, t, p->mass);

		case TRAJECTORY_STATE_FLYING:
			return piecewise_eval(p->ppFront, t, p->mass);

		default:
			return traj_eval_invalid();
	}
}


// for piecewise trajectories, build the piecewise polynomial yourself
// in planner->ppBack, then call this function.
//
// this function shifts the polynomial in ppBack
// so it starts at the current position.
//
void plan_start_poly(struct planner *p, struct vec current_pos, float t)
{
	struct traj_eval traj_init = poly4d_eval(&p->ppBack->pieces[0], 0, p->mass);
	struct vec shift_pos = vsub(current_pos, traj_init.pos);
	piecewise_shift_vec(p->ppBack, shift_pos, 0);
	plan_pp_flip(p, t);
}

int plan_start_canned_trajectory(struct planner *p, enum trajectory_type type,
	float timescale, struct vec current_pos, float t)
{
	switch (type) {
	case TRAJECTORY_FIGURE8:
		*p->ppBack = pp_figure8;
		break;
	default:
		return 1;
	}

	piecewise_stretchtime(p->ppBack, timescale);
	plan_start_poly(p, current_pos, t);
	return 0;
}

// build the ellipse yourself in planner->ellipse, then call this function.
//
void plan_start_ellipse(struct planner *p, float t)
{
	if (p->state != TRAJECTORY_STATE_ELLIPSE_CATCHUP) {
		p->ellipse.t_begin = t;
		struct traj_eval ev_current = plan_current_goal(p, t);
		plan_into_ellipse(&ev_current, &p->ellipse, p->ppBack, p->mass);
		plan_pp_flip(p, t);
		p->state = TRAJECTORY_STATE_ELLIPSE_CATCHUP;
	}
}

int plan_takeoff(struct planner *p, struct vec pos, float yaw, float height, float duration, float t)
{
	if (p->state != TRAJECTORY_STATE_IDLE) {
		return 1;
	}

	p->home = vadd(pos, mkvec(0, 0, height));
	plan_takeoff_or_landing(p, pos, yaw, height, duration);
	p->state = TRAJECTORY_STATE_TAKING_OFF;
	plan_pp_flip(p, t);
	return 0;
}

int plan_land(struct planner *p, struct vec pos, float yaw, float height, float duration, float t)
{
	// TODO this means we can't land from ELLIPSE or AVOID_TARGET
	// states... you must GO_HOME first. Is this good?
	if (p->state != TRAJECTORY_STATE_FLYING) {
		return 1;
	}

	plan_takeoff_or_landing(p, pos, yaw, height, duration);
	p->state = TRAJECTORY_STATE_LANDING;
	plan_pp_flip(p, t);
	return 0;
}

int plan_hover(struct planner *p, struct vec hover_pos, float hover_yaw, float duration, float t)
{
	if (p->state != TRAJECTORY_STATE_FLYING &&
	    p->state != TRAJECTORY_STATE_ELLIPSE &&
	    p->state != TRAJECTORY_STATE_AVOID_TARGET) {
		return 1;
	}

	struct traj_eval setpoint = plan_current_goal(p, t);

	piecewise_plan_7th_order_no_jerk(p->ppBack, duration,
		setpoint.pos, setpoint.yaw, setpoint.vel, setpoint.omega.z, setpoint.acc,
		hover_pos,    hover_yaw,    vzero(),      0,                vzero());

	p->state = TRAJECTORY_STATE_FLYING;
	plan_pp_flip(p, t);
	return 0;
}

int plan_go_home(struct planner *p, float t)
{
	struct traj_eval setpoint = plan_current_goal(p, t);
	float distance = vdist(p->home, setpoint.pos);
	float duration = 1.0f + distance;
	float yaw = 0;
	return plan_hover(p, p->home, yaw, duration, t);
}

// TODO make user build p->avoid manually, like for pp and ellipse?
void plan_start_avoid_target(
	struct planner *p, struct vec home, float max_displacement, float max_speed, float t)
{
	if (p->state != TRAJECTORY_STATE_AVOID_TARGET) {
		init_avoid_target(&p->avoid, home, max_speed, max_displacement, t);
		p->state = TRAJECTORY_STATE_AVOID_TARGET;
	}
}

void plan_update_avoid_target(struct planner *p, struct vec target_pos, float t)
{
	update_avoid_target(&p->avoid, target_pos, t);
}
