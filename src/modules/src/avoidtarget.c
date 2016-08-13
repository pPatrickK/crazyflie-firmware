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

#include "avoidtarget.h"
#include "pptraj.h"
//#include "stdio.h"

static float dist_to_velocity_poly[PP_SIZE];
static float const DIST_OF_MAX_VELOCITY = 0.7;

void init_avoid_target(struct avoid_target *at, struct vec home, float max_speed, float max_displace, float t)
{
	at->home = home;
	at->pos = home;
	at->vel = vzero();
	at->max_speed = max_speed;
	at->max_displace = max_displace;
	at->last_t = t;

	poly5(dist_to_velocity_poly, DIST_OF_MAX_VELOCITY,
		0, 0, 0, max_speed, 0, 0);
}

void update_avoid_target(struct avoid_target *at, struct vec target_pos, float t)
{
	// this is an xy controller only... for now
	target_pos.z = at->home.z;

	//printf("update: target pos (%f %f %f)\n", target_pos.x, target_pos.y, target_pos.z);
	//printf("            my pos (%f %f %f)\n", at->pos.x, at->pos.y, at->pos.z);
	//printf("      max_displace %f\n", at->max_displace);
	//printf("         max_speed %f\n", at->max_speed);

	// compute goal position
	// this should really be relative to at->home instead of at->pos,
	// but for some reason this gives a nicer, more smooth + organic movement -
	// TODO figure out why
	struct vec delta = vsub(at->pos, target_pos);
	float dist = vmag(delta);
	struct vec delta_unit = vdiv(delta, dist);
	// d + d^2 for strong repulsion when near, but also long tail
	if (dist < 0.001) {
		// avoid div by 0 (target is too close)
		at->vel = vzero();
		at->last_t = t;
		return;
	}
	// TODO: prove no collisions
	float desired_displacement = 1.5 / (dist + fsqr(dist));
	float displacement = fmin(desired_displacement, at->max_displace);
	struct vec goal = vadd(at->home, vscl(displacement, delta_unit));
	//printf("              goal (%f %f %f)\n", goal.x, goal.y, goal.z);

	// move towards goal position
	struct vec pos2goal = vsub(goal, at->pos);
	float dist2goal = vmag(pos2goal);
	if (dist2goal < 0.001) {
		// avoid div by 0 (target is very far) (target is very far)
		at->vel = vzero();
		at->last_t = t;
		return;
	}
	float speed = fmin(polyval(dist_to_velocity_poly, dist2goal), at->max_speed);

	float dt = t - at->last_t;
	at->vel = vscl(speed / dist2goal, pos2goal);
	at->pos = vadd(at->pos, vscl(dt, at->vel));
	at->last_t = t;
}

struct traj_eval eval_avoid_target(struct avoid_target *at, float t)
{
	float dt = t - at->last_t;
	at->pos = vadd(at->pos, vscl(dt, at->vel));
	at->last_t = t;

	struct traj_eval ev;
	ev.pos = at->pos;
	ev.vel = at->vel;
	ev.acc = vzero();
	ev.yaw = 0;
	ev.omega = vzero();
	// TODO should we supply acceleration?
	return ev;
}
