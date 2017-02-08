#pragma once

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

#include "avoidtarget.h"
#include "math3d.h"
#include "packetdef.h"
#include "pptraj.h"


enum trajectory_state
{
	TRAJECTORY_STATE_IDLE            = 0,
	TRAJECTORY_STATE_FLYING          = 1,
	TRAJECTORY_STATE_TAKING_OFF      = 2,
	TRAJECTORY_STATE_LANDING         = 3,
	TRAJECTORY_STATE_ELLIPSE         = 4,
	TRAJECTORY_STATE_ELLIPSE_CATCHUP = 5,
	TRAJECTORY_STATE_AVOID_TARGET    = 6,
};

struct planner
{
	enum trajectory_state state;
	enum trajectory_direction direction;

	struct piecewise_traj* ppFront;
	struct piecewise_traj* ppBack;
	struct piecewise_traj pp1;
	struct piecewise_traj pp2;

	struct ellipse_traj ellipse;

	struct avoid_target avoid;

	struct vec home;

	float mass;
};

// initialize the planner. pass in the vehicle mass in kilograms.
void plan_init(struct planner *p, float mass);

// tell the planner that something bad has happened.
// subsequently, plan_is_stopped(p) will return true,
// and it is no longer valid to call plan_current_goal(p).
void plan_emergency_stop(struct planner *p);

// query if the planner is stopped.
// currently this is true at startup before we take off,
// and also after an emergency stop.
bool plan_is_stopped(struct planner *p);

// query if the planner is flying.
// this is similar to plan_is_stopped, with the exception
// that takeoff and landing do not count as flying.
bool plan_is_flying(struct planner *p);

// get the planner's current goal.
struct traj_eval plan_current_goal(struct planner *p, float t);

// for piecewise trajectories, build the piecewise polynomial yourself
// in planner->ppBack, then call this function.
//
// this function shifts the polynomial in ppBack
// so it starts at the current position.
//
void plan_start_poly(struct planner *p, struct vec current_pos, float t,
	enum trajectory_direction direction);

int plan_start_canned_trajectory(struct planner *p, enum trajectory_type type,
	float timescale, struct vec current_pos, float t);

// build the ellipse yourself in planner->ellipse, then call this function.
//
void plan_start_ellipse(struct planner *p, float t);

// start a takeoff trajectory.
int plan_takeoff(struct planner *p, struct vec pos, float yaw, float height, float duration, float t);

// start a landing trajectory.
int plan_land(struct planner *p, struct vec pos, float yaw, float height, float duration, float t);

// move to a given position, then stay there.
int plan_hover(struct planner *p, struct vec hover_pos, float hover_yaw, float duration, float t);

// go to the position we first took off to.
int plan_go_home(struct planner *p, float t);

// enter "avoid target" interactive mode.
// TODO make user build p->avoid manually, like for pp and ellipse?
void plan_start_avoid_target(
	struct planner *p, struct vec home, float max_displacement, float max_speed, float t);

// update the target's position while in "avoid target" mode.
void plan_update_avoid_target(struct planner *p, struct vec target_pos, float t);
