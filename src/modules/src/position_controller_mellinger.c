/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2016 Bitcraze AB
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
 * position_controller_mellinger.c: Position controller based on Mellingers work
 */

#include <math.h>

// #include "commander.h"
// #include "log.h"
#include "param.h"
// #include "pid.h"
// #include "num.h"
#include "position_controller.h"

#define DT 0.01

typedef struct {
  float x;
  float y;
  float z;
} vector3f;

// computes the dot-product between the two vectors
static float dot(
  const vector3f* a,
  const vector3f* b)
{
  return a->x * b->x + a->y * b->y + a->z * b->z;
}
// computes vector norm
static float norm(
  const vector3f* a)
{
  return sqrt(pow(a->x, 2) + pow(a->y, 2) + pow(a->z, 2));
}

static void normalize(
  vector3f* a)
{
  float n = norm(a);
  if (n > 0) {
    a->x /= n;
    a->y /= n;
    a->z /= n;
  }
}

// c = axb
static void cross(
  const vector3f* a,
  const vector3f* b,
  vector3f* c)
{
  c->x = a->y * b->z - a->z * b->y;
  c->y = a->z * b->x - a->x * b->z;
  c->z = a->x * b->y - a->y * b->x;
}


static const float g = 9.81;
static const float mass = 0.030;

static float kp_xy = 0.2;
static float kd_xy = 0.1;
static float ki_xy = 0.05;

static float kp_z = 0.2;
static float kd_z = 0.1;
static float ki_z = 0.05;

static float massThrust = 130500;

static float i_error_x = 0;
static float i_error_y = 0;
static float i_error_z = 0;


void positionControllerReset(void)
{
  i_error_x = 0;
  i_error_y = 0;
  i_error_z = 0;
}

void positionController(
  float *thrust,
  attitude_t *attitude,
  const state_t *state,
  const setpoint_t *setpoint)
{
  vector3f r_error;
  vector3f v_error;
  vector3f target_thrust;
  vector3f z_axis;
  float current_thrust;
  vector3f z_axis_desired;
  vector3f x_axis_desired;
  vector3f y_axis_desired;
  vector3f x_c_des;

  // Position Error
  r_error.x = setpoint->position.x - state->position.x;
  r_error.y = setpoint->position.y - state->position.y;
  r_error.z = setpoint->position.z - state->position.z;

  // Velocity Error
  v_error.x = setpoint->velocity.x - state->velocity.x;
  v_error.y = setpoint->velocity.y - state->velocity.y;
  v_error.z = setpoint->velocity.z - state->velocity.z;

  // Integral Error
  i_error_z += r_error.z * DT;
  if (i_error_z < -0.5) {
    i_error_z = -0.5;
  }
  if (i_error_z > 0.5) {
    i_error_z = 0.5;
  }

  i_error_x += r_error.x * DT;
  if (i_error_x < -0.5) {
    i_error_x = -0.5;
  }
  if (i_error_x > 0.5) {
    i_error_x = 0.5;
  }

  i_error_y += r_error.y * DT;
  if (i_error_y < -0.5) {
    i_error_y = -0.5;
  }
  if (i_error_y > 0.5) {
    i_error_y = 0.5;
  }

  // Desired thrust (ignoring target accellerations)
  target_thrust.x = kp_xy * r_error.x + kd_xy * v_error.x + mass * 0 + ki_xy * i_error_x;
  target_thrust.y = kp_xy * r_error.y + kd_xy * v_error.y + mass * 0 + ki_xy * i_error_y;
  target_thrust.z = kp_z  * r_error.z + kd_z  * v_error.z + mass * g + ki_z  * i_error_z;

  // Z-Axis
  z_axis.x = -sin(state->attitude.pitch / 180 * M_PI) * cos(state->attitude.roll / 180 * M_PI);
  z_axis.y = sin(state->attitude.roll) / 180 * M_PI;
  z_axis.z = cos(state->attitude.pitch / 180 * M_PI) * cos(state->attitude.roll / 180 * M_PI);

  // Current thrust
  current_thrust = massThrust * dot(&target_thrust, &z_axis);

  // Calculate axis
  z_axis_desired = target_thrust;
  normalize(&z_axis_desired);

  // x_axis_desired = z_axis_desired x [sin(yaw), cos(yaw), 0]^T
  x_c_des.x = cos(setpoint->attitude.yaw / 180 * M_PI);
  x_c_des.y = sin(setpoint->attitude.yaw / 180 * M_PI);
  x_c_des.z = 0;
  cross(&z_axis_desired, &x_c_des, &y_axis_desired);
  normalize(&y_axis_desired);

  cross(&y_axis_desired, &z_axis_desired, &x_axis_desired);
  // cross(&z_axis_desired, &tmp, &x_axis_desired);

  // y_axis_desired = z_axis_desired x x_axis_desired
  // cross(&z_axis_desired, &x_axis_desired, &y_axis_desired);

  // Output
  attitude->roll = atan2(y_axis_desired.z, z_axis_desired.z) * 180.0 / M_PI;
  attitude->pitch = asin(x_axis_desired.z) * 180.0 / M_PI;
  attitude->yaw = setpoint->attitude.yaw;
  *thrust = current_thrust;
}

PARAM_GROUP_START(ctrlMel)
PARAM_ADD(PARAM_FLOAT, kp_xy, &kp_xy)
PARAM_ADD(PARAM_FLOAT, kd_xy, &kd_xy)
PARAM_ADD(PARAM_FLOAT, kp_z, &kp_z)
PARAM_ADD(PARAM_FLOAT, kd_z, &kd_z)
PARAM_ADD(PARAM_FLOAT, mass, &mass)
PARAM_ADD(PARAM_FLOAT, massThrust, &massThrust)
PARAM_GROUP_STOP(ctrlMel)
