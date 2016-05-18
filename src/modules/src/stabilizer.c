/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
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
 */
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "system.h"
#include "log.h"
#include "param.h"

#include "stabilizer.h"

#include "sensors.h"
#include "estimator.h"
#include "commander.h"
#include "sitaw.h"
#include "controller.h"
#include "power_distribution.h"
#include "position_external_bringup.h"
#include "SEGGER_RTT.h"

#include "ekf.h"
#include "trajectory.h"
#include "debug.h"

static bool isInit;

// State variables for the stabilizer
static setpoint_t setpoint;
static sensorData_t sensorData;
static state_t state;
static float roll_rad, pitch_rad, yaw_rad;
static control_t control;

static struct ekf ekfa;
static struct ekf ekfb;
struct vec ekf_est_pos;
struct vec ekf_est_rpy;

static void stabilizerTask(void* param);

void stabilizerInit(void)
{
  if(isInit)
    return;

  sensorsInit();
  stateEstimatorInit();
  stateControllerInit();
  powerDistributionInit();
#if defined(SITAW_ENABLED)
  sitAwInit();
#endif
  positionExternalBringupInit();
  trajectoryInit();

  xTaskCreate(stabilizerTask, STABILIZER_TASK_NAME,
              STABILIZER_TASK_STACKSIZE, NULL, STABILIZER_TASK_PRI, NULL);

  isInit = true;
}

bool stabilizerTest(void)
{
  bool pass = true;

  pass &= sensorsTest();
  pass &= stateEstimatorTest();
  pass &= stateControllerTest();
  pass &= powerDistributionTest();
  pass &= positionExternalBringupTest();
  pass &= trajectoryTest();

  return pass;
}

/* The stabilizer loop runs at 1kHz. It is the responsability or the different
 * functions to run slower by skipping call (ie. returning without modifying
 * the output structure).
 */
static void stabilizerTask(void* param)
{
  uint32_t tick = 0;
  uint32_t lastWakeTime;
  struct ekf *ekf_front = &ekfa;
  struct ekf *ekf_back = &ekfb;
  struct ekf *ekf_tmp;

  vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);

  //Wait for the system to be fully started to start stabilization loop
  systemWaitStart();

  // Wait for sensors to be calibrated
  lastWakeTime = xTaskGetTickCount ();
  while(!sensorsAreCalibrated()) {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
  }

  // initialize ekf
  while(true)
  {
      float x, y, z, q0, q1, q2, q3;
      uint16_t last_time_in_ms;
      positionExternalBringupGetLastData(&x, &y, &z, &q0, &q1, &q2, &q3, &last_time_in_ms);
      if (last_time_in_ms <= 10) {
        float pos[3] = {x, y, z};
        float vel[3] = {0, 0, 0};
        float quat[4] = {q0, q1, q2, q3};

        ekf_init(ekf_back, pos, vel, quat);
        break;
      }
      vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
  }

  DEBUG_PRINT("ekf initialized!\n");

  // float pos[3] = {0, 0, 0};
  // float vel[3] = {0, 0, 0};
  // float quat[4] = {0, 0, 0, 1};
  // ekf_init(ekf_back, pos, vel, quat);

  while(1) {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));

    sensorsAcquire(&sensorData, tick);

    if (RATE_DO_EXECUTE(RATE_500_HZ, tick)) {
      float acc[3] = {sensorData.acc.x * GRAV, sensorData.acc.y * GRAV, sensorData.acc.z * GRAV};
      float gyro[3] = {sensorData.gyro.x * M_PI / 180.0, sensorData.gyro.y * M_PI / 180.0, sensorData.gyro.z * M_PI / 180.0};
      ekf_imu(ekf_back, ekf_front, acc, gyro, 1.0 / RATE_500_HZ);
      // swap ptr
      ekf_tmp = ekf_front;
      ekf_front = ekf_back;
      ekf_back = ekf_tmp;

      float x, y, z, q0, q1, q2, q3;
      uint16_t last_time_in_ms;
      positionExternalBringupGetLastData(&x, &y, &z, &q0, &q1, &q2, &q3, &last_time_in_ms);

      sensorData.valid = last_time_in_ms < 1000;
      struct quat quat_tmp = mkquat(q0, q1, q2, q3);
      struct vec tmp = quat2rpy(quat_tmp);
      sensorData.external_yaw = tmp.z;

      //DEBUG_PRINT("quat: %f,%f,%f,%f; %f\n", quat_tmp.x, quat_tmp.y, quat_tmp.z, quat_tmp.w, degrees(tmp.z));

      // check if new vicon data available
      if (positionExternalBringupFresh) {
        float pos_vicon[3] = {x, y, z};
        float quat_vicon[4] = {q0, q1, q2, q3};

        ekf_vicon(ekf_back, ekf_front, pos_vicon, quat_vicon);
        // swap ptr
        ekf_tmp = ekf_front;
        ekf_front = ekf_back;
        ekf_back = ekf_tmp;

        positionExternalBringupFresh = false;
      }

	  ekf_est_pos = ekf_back->pos;
	  ekf_est_rpy = quat2rpy(qinv(ekf_back->quat));

      // Here we do the print out for datacollection:
      uint32_t time = xTaskGetTickCount();
      uint16_t magic = 0xFBCF;

      // SEGGER_RTT_Write(0, (const char*)&magic, sizeof(magic));
      // SEGGER_RTT_Write(0, (const char*)&time, sizeof(tick));
      // SEGGER_RTT_Write(0, (const char*)&ekf_back->pos, sizeof(ekf_back->pos));
      // // SEGGER_RTT_Write(0, (const char*)&sensorData.gyro, sizeof(sensorData.gyro));
      // // SEGGER_RTT_Write(0, (const char*)&sensorData.acc, sizeof(sensorData.acc));
      // SEGGER_RTT_Write(0, (const char*)&x, sizeof(x));
      // SEGGER_RTT_Write(0, (const char*)&y, sizeof(y));
      // SEGGER_RTT_Write(0, (const char*)&z, sizeof(z));
      // SEGGER_RTT_Write(0, (const char*)&q0, sizeof(q0));
      // SEGGER_RTT_Write(0, (const char*)&q1, sizeof(q1));
      // SEGGER_RTT_Write(0, (const char*)&q2, sizeof(q2));
      // SEGGER_RTT_Write(0, (const char*)&q3, sizeof(q3));
      // SEGGER_RTT_Write(0, (const char*)&last_time_in_ms, sizeof(last_time_in_ms));
    }



    stateEstimator(&state, &sensorData, tick);

    // HACK: update state from ekf
    roll_rad = radians(state.attitude.roll);
    pitch_rad = radians(state.attitude.pitch);
    yaw_rad = radians(state.attitude.yaw);

    state.position.x = ekf_back->pos.x;
    state.position.y = ekf_back->pos.y;
    state.position.z = ekf_back->pos.z;
    state.velocity.x = ekf_back->vel.x;
    state.velocity.y = ekf_back->vel.y;
    state.velocity.z = ekf_back->vel.z;

    state.attitude.roll = degrees(ekf_est_rpy.x);
    state.attitude.pitch = -degrees(ekf_est_rpy.y);
    state.attitude.yaw = degrees(ekf_est_rpy.z);

    commanderGetSetpoint(&setpoint, &state);

    // TODO: Disabled for now to avoid any side-effects
    //       Check if we can enable this again.
    // sitAwUpdateSetpoint(&setpoint, &sensorData, &state);



    stateController(&control, &sensorData, &state, &setpoint, tick);
    powerDistribution(&control);

    tick++;
  }
}

LOG_GROUP_START(ctrltarget)
LOG_ADD(LOG_FLOAT, roll, &setpoint.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &setpoint.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &setpoint.attitudeRate.yaw)
LOG_GROUP_STOP(ctrltarget)

LOG_GROUP_START(stabilizer)
LOG_ADD(LOG_FLOAT, roll, &state.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &state.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &state.attitude.yaw)
LOG_ADD(LOG_FLOAT, roll_rad, &roll_rad)
LOG_ADD(LOG_FLOAT, pitch_rad, &pitch_rad)
LOG_ADD(LOG_FLOAT, yaw_rad, &yaw_rad)
LOG_ADD(LOG_UINT16, thrust, &control.thrust)
LOG_GROUP_STOP(stabilizer)

LOG_GROUP_START(acc)
LOG_ADD(LOG_FLOAT, x, &sensorData.acc.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.acc.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.acc.z)
LOG_GROUP_STOP(acc)

LOG_GROUP_START(baro)
LOG_ADD(LOG_FLOAT, asl, &sensorData.baro.asl)
LOG_GROUP_STOP(baro)

LOG_GROUP_START(gyro)
LOG_ADD(LOG_FLOAT, x, &sensorData.gyro.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.gyro.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.gyro.z)
LOG_GROUP_STOP(gyro)

LOG_GROUP_START(mag)
LOG_ADD(LOG_FLOAT, x, &sensorData.mag.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.mag.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.mag.z)
LOG_GROUP_STOP(mag)

LOG_GROUP_START(ekf_estimate)
LOG_ADD(LOG_FLOAT, x, &ekf_est_pos.x)
LOG_ADD(LOG_FLOAT, y, &ekf_est_pos.y)
LOG_ADD(LOG_FLOAT, z, &ekf_est_pos.z)
LOG_ADD(LOG_FLOAT, roll, &ekf_est_rpy.x)
LOG_ADD(LOG_FLOAT, pitch, &ekf_est_rpy.y)
LOG_ADD(LOG_FLOAT, yaw, &ekf_est_rpy.z)
LOG_GROUP_STOP(ekf_estimate)

LOG_GROUP_START(controller)
LOG_ADD(LOG_INT16, ctr_yaw, &control.yaw)
LOG_GROUP_STOP(controller)
