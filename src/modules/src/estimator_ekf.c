/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
 *
 * EKF author: 
 * James Preiss
 * University of Southern California
 *
 */

#include "ekf.h"
#include "stabilizer_types.h"
#include "position_external_bringup.h"


// EKF implementation uses double-buffered approach
static struct ekf ekfa;
static struct ekf ekfb;
static struct ekf *ekf_front = &ekfa;
static struct ekf *ekf_back = &ekfb;
static void ekf_flip()
{
	struct ekf *ekf_temp = ekf_front;
	ekf_front = ekf_back;
	ekf_back = ekf_temp;
}
static bool initialized = false;

// TODO change EKF funcs to take struct vec, 
// then make a function that wraps positionExternalBringupGetLastData

#define ATTITUDE_UPDATE_RATE RATE_500_HZ
#define ATTITUDE_UPDATE_DT (1.0/ATTITUDE_UPDATE_RATE)

static struct vec3_s ekf2vec(struct vec v)
{
	struct vec3_s v3s = { .x = v.x, .y = v.y, .z = v.z };
	return v3s;
}

// public functions

void stateEstimatorInit(void)
{
	// Initialization is lazy within the state estimator main function
	float init[] = {0, 0, 0, 1};
	ekf_init(ekf_back, init, init, init);
}

bool stateEstimatorTest(void)
{
	// Nothing to test
	return true; 
}

void stateEstimator(state_t *state, const sensorData_t *sensorData, const uint32_t tick)
{
	// lazy initialization - need a Vicon measurement for initial position
	/*
	if (!initialized) {
		if (!sensorData->valid) {
			return;
		}
		// TODO get Vicon from sensorData_t instead of this function call
		float x, y, z, q0, q1, q2, q3;
		uint16_t last_time_in_ms;
		positionExternalBringupGetLastData(&x, &y, &z, &q0, &q1, &q2, &q3, &last_time_in_ms);
		if (last_time_in_ms > 10) {
			return;
		}
		float pos[3] = {x, y, z};
		float vel[3] = {0, 0, 0};
		float quat[4] = {q0, q1, q2, q3};
		ekf_init(ekf_back, pos, vel, quat);
		initialized = true;
	}
	*/

	// rate limit
	if (!RATE_DO_EXECUTE(ATTITUDE_UPDATE_RATE, tick)) {
		return;
	}
	// TODO: should we rate-limit IMU but not vicon, so we have less vicon latency?

	float acc[3] = {sensorData->acc.x * GRAV, sensorData->acc.y * GRAV, sensorData->acc.z * GRAV};
	float gyro[3] = {radians(sensorData->gyro.x), radians(sensorData->gyro.y), radians(sensorData->gyro.z)};
	ekf_imu(ekf_back, ekf_front, acc, gyro, ATTITUDE_UPDATE_DT);
	ekf_flip();

	// check if new vicon data available
	if (positionExternalBringupFresh) {

		float x, y, z, q0, q1, q2, q3;
		uint16_t last_time_in_ms;
		positionExternalBringupGetLastData(&x, &y, &z, &q0, &q1, &q2, &q3, &last_time_in_ms);

		float pos_vicon[3] = {x, y, z};
		float quat_vicon[4] = {q0, q1, q2, q3};
		ekf_vicon(ekf_back, ekf_front, pos_vicon, quat_vicon);
		ekf_flip();

		positionExternalBringupFresh = false;
	}
	
	state->position = ekf2vec(ekf_back->pos);
	state->velocity = ekf2vec(ekf_back->vel);

	struct vec rpy = quat2rpy(ekf_back->quat);
	state->attitude.roll = degrees(rpy.x);
	state->attitude.pitch = -degrees(rpy.y);
	state->attitude.yaw = degrees(rpy.z);
}
