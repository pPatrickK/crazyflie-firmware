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
#include "debug.h"

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
static float ki_xy = 0;//0.05;

static float kp_z = 0.2;
static float kd_z = 0.1;
static float ki_z = 0;//0.05;

static float massThrust = 130500;

static float i_error_x = 0;
static float i_error_y = 0;
static float i_error_z = 0;

// "P" part for moment
static float kR_xy = 7000;
static float kR_z  = 7000;

// "D" part for moment
static float kw_xy = 3000;
static float kw_z = 50;



void positionControllerReset(void)
{
  i_error_x = 0;
  i_error_y = 0;
  i_error_z = 0;
}

float clamp(float value, float min, float max) {
  if (value < min) return min;
  if (value > max) return max;
  return value;
}

void positionControllerMellinger(
  control_t *control,
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

  float roll, pitch, yaw;

  vector3f eR, ew, M;

  roll = state->attitude.roll / 180.0 * M_PI;
  pitch = state->attitude.pitch / 180.0 * M_PI;
  yaw = state->attitude.yaw / 180.0 * M_PI;

  // Position Error (ep)
  r_error.x = setpoint->position.x - state->position.x;
  r_error.y = setpoint->position.y - state->position.y;
  r_error.z = setpoint->position.z - state->position.z;

  // Velocity Error (ev)
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

  // Desired thrust (ignoring target accellerations) [F_des]
  target_thrust.x = sin(setpoint->attitude.pitch / 180 * M_PI);//kp_xy * r_error.x + kd_xy * v_error.x + mass * 0 + ki_xy * i_error_x;
  target_thrust.y = -sin(setpoint->attitude.roll / 180 * M_PI);//kp_xy * r_error.y + kd_xy * v_error.y + mass * 0 + ki_xy * i_error_y;
  target_thrust.z = 1;//kp_z  * r_error.z + kd_z  * v_error.z + mass * g + ki_z  * i_error_z;

  // Z-Axis [zB]
  z_axis.x = -sin(pitch) * cos(roll);
  z_axis.y = sin(roll);
  z_axis.z = cos(pitch) * cos(roll);

  // Current thrust [F]
  current_thrust = dot(&target_thrust, &z_axis);

  // Calculate axis [zB_des]
  z_axis_desired = target_thrust;
  normalize(&z_axis_desired);

  // [xC_des]
  // x_axis_desired = z_axis_desired x [sin(yaw), cos(yaw), 0]^T
  // x_c_des.x = cos(yaw);//cos(setpoint->attitude.yaw / 180 * M_PI);
  // x_c_des.y = sin(yaw);//sin(setpoint->attitude.yaw / 180 * M_PI);
  x_c_des.x = cos(setpoint->attitude.yaw / 180 * M_PI);
  x_c_des.y = sin(setpoint->attitude.yaw / 180 * M_PI);
  x_c_des.z = 0;
  // [yB_des]
  cross(&z_axis_desired, &x_c_des, &y_axis_desired);
  normalize(&y_axis_desired);
  // [xB_des]
  cross(&y_axis_desired, &z_axis_desired, &x_axis_desired);

  // [eR] (see mathematica notebook)
  // TODO: it might be more efficient to actually create the rotation matrix and carray out the matrix multiplication?
  float phi = roll;
  float theta = pitch;
  float psi = yaw;
  eR.x = cos(phi) * (z_axis_desired.y * cos(psi) - y_axis_desired.z * cos(theta) - z_axis_desired.x * sin(psi)) + sin(phi) * (z_axis_desired.z + y_axis_desired.y * cos(psi) * cos(theta) - y_axis_desired.x * cos(theta) * sin(psi)) - (y_axis_desired.x * cos(psi) + y_axis_desired.y * sin(psi)) * sin(theta);
  eR.y = cos(theta) * (x_axis_desired.z * cos(phi) - cos(psi) * (z_axis_desired.x + x_axis_desired.y * sin(phi)) + (-z_axis_desired.y + x_axis_desired.x * sin(phi)) * sin(psi)) + (z_axis_desired.z * cos(phi) + cos(psi) * (x_axis_desired.x - z_axis_desired.y * sin(phi)) + (x_axis_desired.y + z_axis_desired.x * sin(phi)) * sin(psi)) * sin(theta);
  eR.z = y_axis_desired.y * cos(theta) * sin(psi) - cos(phi) * (x_axis_desired.y * cos(psi) - x_axis_desired.x * sin(psi) + y_axis_desired.z * sin(theta)) + cos(psi) * (y_axis_desired.x * cos(theta) + y_axis_desired.y * sin(phi) * sin(theta)) - sin(phi) * (x_axis_desired.z + y_axis_desired.x * sin(psi) * sin(theta));

  // ew
  ew.x = setpoint->attitudeRate.roll - state->attitudeRate.roll;
  ew.y = setpoint->attitudeRate.pitch - state->attitudeRate.pitch;
  ew.z = setpoint->attitudeRate.yaw - state->attitudeRate.yaw;

  // Moment:
  M.x = -kR_xy * eR.x + kw_xy * ew.x;
  M.y = -kR_xy * eR.y + kw_xy * ew.y;
  M.z = -kR_z  * eR.z + kw_z  * ew.z;

  // invert (4.1)
  // float u1 = current_thrust;
  // float u2 = M.x;
  // float u3 = M.y;
  // float u4 = M.z;

  // TODO: this assumes a different frame...
  // I think we can simple use the power_distribution here...
  //  the various constants are then "integrated" into kR and kw
  // float w1 = u1/(4 * kf) - u3/(2 * kf * L) + u4/(4 * kM);
  // float w2 = u1/(4 * kf) + u2/(2 * kf * L) - u4/(4 * kM);
  // float w3 = u1/(4 * kf) + u3/(2 * kf * L) + u4/(4 * kM);
  // float w4 = u1/(4 * kf) - u2/(2 * kf * L) - u4/(4 * kM);



  // Output
  control->thrust = massThrust * current_thrust;
  control->roll = clamp(M.x, -32000, 32000);
  control->pitch = clamp(M.y, -32000, 32000);
  control->yaw = clamp(-M.z, -32000, 32000);

  DEBUG_PRINT("%f,%f,%f,%f,%f,%f,%f\n", control->thrust, roll, pitch, yaw, M.x, M.y, M.z);

  // attitude->roll = atan2(y_axis_desired.z, z_axis_desired.z) * 180.0 / M_PI;
  // attitude->pitch = asin(x_axis_desired.z) * 180.0 / M_PI;
  // attitude->yaw = setpoint->attitude.yaw;
  // *thrust = massThrust * current_thrust;
}

PARAM_GROUP_START(ctrlMel)
PARAM_ADD(PARAM_FLOAT, kp_xy, &kp_xy)
PARAM_ADD(PARAM_FLOAT, kd_xy, &kd_xy)
PARAM_ADD(PARAM_FLOAT, kp_z, &kp_z)
PARAM_ADD(PARAM_FLOAT, kd_z, &kd_z)
PARAM_ADD(PARAM_FLOAT, mass, &mass)
PARAM_ADD(PARAM_FLOAT, massThrust, &massThrust)
PARAM_ADD(PARAM_FLOAT, kR_xy, &kR_xy)
PARAM_ADD(PARAM_FLOAT, kR_z, &kR_z)
PARAM_ADD(PARAM_FLOAT, kw_xy, &kw_xy)
PARAM_ADD(PARAM_FLOAT, kw_z, &kw_z)
PARAM_GROUP_STOP(ctrlMel)
