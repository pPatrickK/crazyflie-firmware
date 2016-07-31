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
 * avoidtarget.h: "avoid target" potential field planner
 */

#include "math3d.h"
#include "pptraj.h"

struct avoid_target
{
	struct vec home;
	struct vec pos;
	struct vec vel;
	float max_speed;
	float max_displace;
	float last_t;
};

// TODO would it be better to lazy-initialize last_t in update or eval?
void init_avoid_target(struct avoid_target *at, struct vec home, float max_speed, float max_displace, float t);

// vicon measurement
void update_avoid_target(struct avoid_target *at, struct vec target_pos, float t);

// get controller setpoint
struct traj_eval eval_avoid_target(struct avoid_target *at, float t);
