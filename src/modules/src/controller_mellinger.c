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
#include "log.h"
#include "math3d.h"
#include "mathconstants.h"


// HACK HACK HACK global for others to use
float g_vehicleMass = 0.033;

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
static float kR_xy = 9000;
static float kR_z  = 7000;

// "D" part for moment
static float kw_xy = 4000;
static float kw_z = 3000;

// "D" part for angular velocity
static float kd_omega_rp = 1000;
static float prev_omega_roll;
static float prev_omega_pitch;
static float prev_setpoint_omega_roll;
static float prev_setpoint_omega_pitch;

// "I" part for moment
static float ki_m_xy = 0.0;
static float ki_m_z = 0.0;
  //
static float i_error_m_x = 0;
static float i_error_m_y = 0;
static float i_error_m_z = 0;
static float i_range_m_xy = 1.0;
static float i_range_m_z  = 2.0;

static float i_range_xy = 2.0;
static float i_range_z  = 1.0;

static struct vec z_axis_desired;

void stateControllerInit(void)
{
}

bool stateControllerTest(void)
{
  return true;
}

void stateControllerReset(void)
{
  i_error_x = 0;
  i_error_y = 0;
  i_error_z = 0;
  i_error_m_x = 0;
  i_error_m_y = 0;
  i_error_m_z = 0;
}

float clamp(float value, float min, float max) {
  if (value < min) return min;
  if (value > max) return max;
  return value;
}

// void positionControllerMellinger(
//   control_t *control,
//   const state_t *state,
//   const setpoint_t *setpoint,
//   float dt)
void stateController(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  struct vec r_error;
  // struct vec v_error;
  struct vec target_thrust;
  struct vec z_axis;
  float current_thrust;
  // struct vec z_axis_desired;
  struct vec x_axis_desired;
  struct vec y_axis_desired;
  struct vec x_c_des;

  //float roll, pitch, yaw;

  struct vec eR, ew, M;

  float dt;

  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    return;
  }

  dt = (float)(1.0f/ATTITUDE_RATE);

  //roll = state->attitude.roll / 180.0 * M_PI;
  //pitch = state->attitude.pitch / 180.0 * M_PI;
  //yaw = state->attitude.yaw / 180.0 * M_PI;

  // Position Error (ep)
  r_error.x = setpoint->position.x - state->position.x;
  r_error.y = setpoint->position.y - state->position.y;
  r_error.z = setpoint->position.z - state->position.z;

  // Velocity Error (ev)
  // v_error.x = setpoint->velocity.x - state->velocity.x;
  // v_error.y = setpoint->velocity.y - state->velocity.y;
  // v_error.z = setpoint->velocity.z - state->velocity.z;

  // Integral Error
  i_error_z += r_error.z * dt;
  i_error_z = clamp(i_error_z, -i_range_z, i_range_z);

  i_error_x += r_error.x * dt;
  i_error_x = clamp(i_error_x, -i_range_xy, i_range_xy);

  i_error_y += r_error.y * dt;
  i_error_y = clamp(i_error_y, -i_range_xy, i_range_xy);

  // Desired thrust [F_des]

  // if (setpoint->enablePosCtrl) {
  //   target_thrust.x = g_vehicleMass * setpoint->acceleration.x          + kp_xy * r_error.x + kd_xy * v_error.x + ki_xy * i_error_x;
  //   target_thrust.y = g_vehicleMass * setpoint->acceleration.y          + kp_xy * r_error.y + kd_xy * v_error.y + ki_xy * i_error_y;
  //   target_thrust.z = g_vehicleMass * (setpoint->acceleration.z + GRAV) + kp_z  * r_error.z + kd_z  * v_error.z + ki_z  * i_error_z;
  // } else {
    target_thrust.x = -sin(setpoint->attitude.pitch / 180 * M_PI);
    target_thrust.y = -sin(setpoint->attitude.roll / 180 * M_PI);
    target_thrust.z = 1;
  // }

  // Z-Axis [zB]
  struct quat q = mkquat(state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z, state->attitudeQuaternion.w);
  struct mat33 R = quat2rotmat(q);
  z_axis = mcolumn(R, 2);

  // z_axis.x = -sin(pitch) * cos(roll);
  // z_axis.y = -sin(roll);
  // z_axis.z = cos(pitch) * cos(roll);

  // Current thrust [F]
  current_thrust = vdot(target_thrust, z_axis);

  // Calculate axis [zB_des]
  z_axis_desired = vnormalize(target_thrust);

  // [xC_des]
  // x_axis_desired = z_axis_desired x [sin(yaw), cos(yaw), 0]^T
  // x_c_des.x = cos(yaw);//cos(setpoint->attitude.yaw / 180 * M_PI);
  // x_c_des.y = sin(yaw);//sin(setpoint->attitude.yaw / 180 * M_PI);
  x_c_des.x = cos(setpoint->attitude.yaw / 180 * M_PI);
  x_c_des.y = sin(setpoint->attitude.yaw / 180 * M_PI);
  x_c_des.z = 0;
  // [yB_des]
  y_axis_desired = vnormalize(vcross(z_axis_desired, x_c_des));
  // [xB_des]
  x_axis_desired = vcross(y_axis_desired, z_axis_desired);

  // [eR] (see mathematica notebook)
  // TODO: it might be more efficient to actually create the rotation matrix and carray out the matrix multiplication?
  // float phi = roll;
  // float theta = pitch;
  // float psi = yaw;
  // eR.x = cos(phi) * (z_axis_desired.y * cos(psi) - y_axis_desired.z * cos(theta) - z_axis_desired.x * sin(psi)) + sin(phi) * (z_axis_desired.z + y_axis_desired.y * cos(psi) * cos(theta) - y_axis_desired.x * cos(theta) * sin(psi)) - (y_axis_desired.x * cos(psi) + y_axis_desired.y * sin(psi)) * sin(theta);
  // eR.y = cos(theta) * (x_axis_desired.z * cos(phi) - cos(psi) * (z_axis_desired.x + x_axis_desired.y * sin(phi)) + (-z_axis_desired.y + x_axis_desired.x * sin(phi)) * sin(psi)) + (z_axis_desired.z * cos(phi) + cos(psi) * (x_axis_desired.x - z_axis_desired.y * sin(phi)) + (x_axis_desired.y + z_axis_desired.x * sin(phi)) * sin(psi)) * sin(theta);
  // eR.z = y_axis_desired.y * cos(theta) * sin(psi) - cos(phi) * (x_axis_desired.y * cos(psi) - x_axis_desired.x * sin(psi) + y_axis_desired.z * sin(theta)) + cos(psi) * (y_axis_desired.x * cos(theta) + y_axis_desired.y * sin(phi) * sin(theta)) - sin(phi) * (x_axis_desired.z + y_axis_desired.x * sin(psi) * sin(theta));


  // SLOW VERSION
  // struct mat33 Rdes = mcolumns(
  //   mkvec(x_axis_desired.x, x_axis_desired.y, x_axis_desired.z),
  //   mkvec(y_axis_desired.x, y_axis_desired.y, y_axis_desired.z),
  //   mkvec(z_axis_desired.x, z_axis_desired.y, z_axis_desired.z));

  // struct mat33 R_transpose = mtranspose(R);
  // struct mat33 Rdes_transpose = mtranspose(Rdes);

  // struct mat33 eRM = msub(mmult(Rdes_transpose, R), mmult(R_transpose, Rdes));

  // eR.x = eRM.m[2][1];
  // eR.y = -eRM.m[0][2];
  // eR.z = eRM.m[1][0];

  // FAST VERSION
  float x = q.x;
  float y = q.y;
  float z = q.z;
  float w = q.w;
  eR.x = (-1 + 2*pow(x,2) + 2*pow(y,2))*y_axis_desired.z + z_axis_desired.y - 2*(x*y_axis_desired.x*z + y*y_axis_desired.y*z - x*y*z_axis_desired.x + pow(x,2)*z_axis_desired.y + pow(z,2)*z_axis_desired.y - y*z*z_axis_desired.z) +    2*w*(-(y*y_axis_desired.x) - z*z_axis_desired.x + x*(y_axis_desired.y + z_axis_desired.z));
  eR.y = x_axis_desired.z - z_axis_desired.x - 2*(pow(x,2)*x_axis_desired.z + y*(x_axis_desired.z*y - x_axis_desired.y*z) - (pow(y,2) + pow(z,2))*z_axis_desired.x + x*(-(x_axis_desired.x*z) + y*z_axis_desired.y + z*z_axis_desired.z) + w*(x*x_axis_desired.y + z*z_axis_desired.y - y*(x_axis_desired.x + z_axis_desired.z)));
  eR.z = y_axis_desired.x - 2*(y*(x*x_axis_desired.x + y*y_axis_desired.x - x*y_axis_desired.y) + w*(x*x_axis_desired.z + y*y_axis_desired.z)) + 2*(-(x_axis_desired.z*y) + w*(x_axis_desired.x + y_axis_desired.y) + x*y_axis_desired.z)*z - 2*y_axis_desired.x*pow(z,2) + x_axis_desired.y*(-1 + 2*pow(x,2) + 2*pow(z,2));

  // ?????
  eR.y = -eR.y;

  // ew
  float err_d_roll = 0;
  float err_d_pitch = 0;

  float stateAttitudeRateRoll = sensors->gyro.x / 180.0 * M_PI;
  float stateAttitudeRatePitch = -sensors->gyro.y / 180.0 * M_PI;
  float stateAttitudeRateYaw = sensors->gyro.z / 180.0 * M_PI;


  ew.x = setpoint->attitudeRate.roll - stateAttitudeRateRoll;
  ew.y = -setpoint->attitudeRate.pitch - stateAttitudeRatePitch;
  ew.z = setpoint->attitudeRate.yaw - stateAttitudeRateYaw;
  if (prev_omega_roll == prev_omega_roll) { /*d part initialized*/
    err_d_roll = ((setpoint->attitudeRate.roll - prev_setpoint_omega_roll) - (stateAttitudeRateRoll - prev_omega_roll)) / dt;
    err_d_pitch = (-(setpoint->attitudeRate.pitch - prev_setpoint_omega_pitch) - (stateAttitudeRatePitch - prev_omega_pitch)) / dt;
  }
  prev_omega_roll = stateAttitudeRateRoll;
  prev_omega_pitch = stateAttitudeRatePitch;
  prev_setpoint_omega_roll = setpoint->attitudeRate.roll;
  prev_setpoint_omega_pitch = setpoint->attitudeRate.pitch;

  // Integral Error
  i_error_m_x += (-eR.x) * dt;
  i_error_m_x = clamp(i_error_m_x, -i_range_m_xy, i_range_m_xy);

  i_error_m_y += (-eR.y) * dt;
  i_error_m_y = clamp(i_error_m_y, -i_range_m_xy, i_range_m_xy);

  i_error_m_z += (-eR.z) * dt;
  i_error_m_z = clamp(i_error_m_z, -i_range_m_z, i_range_m_z);

  // Moment:
  M.x = -kR_xy * eR.x + kw_xy * ew.x + ki_m_xy * i_error_m_x + kd_omega_rp * err_d_roll;
  M.y = -kR_xy * eR.y + kw_xy * ew.y + ki_m_xy * i_error_m_y + kd_omega_rp * err_d_pitch;
  M.z = -kR_z  * eR.z + kw_z  * ew.z + ki_m_z  * i_error_m_z;

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

  // DEBUG_PRINT("%f,%f,%f,%f,%f,%f,%f\n", control->thrust, roll, pitch, yaw, M.x, M.y, M.z);

  // attitude->roll = atan2(y_axis_desired.z, z_axis_desired.z) * 180.0 / M_PI;
  // attitude->pitch = asin(x_axis_desired.z) * 180.0 / M_PI;
  // attitude->yaw = setpoint->attitude.yaw;
  // *thrust = massThrust * current_thrust;
}

PARAM_GROUP_START(ctrlMel)
PARAM_ADD(PARAM_FLOAT, kp_xy, &kp_xy)
PARAM_ADD(PARAM_FLOAT, kd_xy, &kd_xy)
PARAM_ADD(PARAM_FLOAT, ki_xy, &ki_xy)
PARAM_ADD(PARAM_FLOAT, i_range_xy, &i_range_xy)
PARAM_ADD(PARAM_FLOAT, kp_z, &kp_z)
PARAM_ADD(PARAM_FLOAT, kd_z, &kd_z)
PARAM_ADD(PARAM_FLOAT, ki_z, &ki_z)
PARAM_ADD(PARAM_FLOAT, i_range_z, &i_range_z)
PARAM_ADD(PARAM_FLOAT, mass, &g_vehicleMass)
PARAM_ADD(PARAM_FLOAT, massThrust, &massThrust)
PARAM_ADD(PARAM_FLOAT, kR_xy, &kR_xy)
PARAM_ADD(PARAM_FLOAT, kR_z, &kR_z)
PARAM_ADD(PARAM_FLOAT, kw_xy, &kw_xy)
PARAM_ADD(PARAM_FLOAT, kw_z, &kw_z)
PARAM_ADD(PARAM_FLOAT, ki_m_xy, &ki_m_xy)
PARAM_ADD(PARAM_FLOAT, ki_m_z, &ki_m_z)
PARAM_ADD(PARAM_FLOAT, kd_omega_rp, &kd_omega_rp)
PARAM_ADD(PARAM_FLOAT, i_range_m_xy, &i_range_m_xy)
PARAM_ADD(PARAM_FLOAT, i_range_m_z, &i_range_m_z)
PARAM_GROUP_STOP(ctrlMel)

LOG_GROUP_START(ctrlMel)
LOG_ADD(LOG_FLOAT, zdx, &z_axis_desired.x)
LOG_ADD(LOG_FLOAT, zdy, &z_axis_desired.y)
LOG_ADD(LOG_FLOAT, zdz, &z_axis_desired.z)
LOG_ADD(LOG_FLOAT, i_err_x, &i_error_x)
LOG_ADD(LOG_FLOAT, i_err_y, &i_error_y)
LOG_ADD(LOG_FLOAT, i_err_z, &i_error_z)
LOG_GROUP_STOP(ctrlMel)
