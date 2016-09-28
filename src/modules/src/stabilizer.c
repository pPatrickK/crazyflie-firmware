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
// #include "controller.h"
#include "power_distribution.h"
#include "position_external.h"
#define USD_LOG_DATA
#ifdef USD_LOG_DATA
  #include "usddeck.h"
#endif

#include "position_controller.h"
#include "SEGGER_RTT.h"
#include "usec_time.h"

#include "trajectory.h"
#include "debug.h"

static bool isInit;

// State variables for the stabilizer
static setpoint_t setpoint;
static sensorData_t sensorData;
static state_t state;
static float roll_rad, pitch_rad, yaw_rad;
static control_t control;

static void stabilizerTask(void* param);

static float m_roll;
static float m_pitch;
static float m_yaw;

static int usec_ekf;
static int usec_traj;
static int usec_ctrl;
static int usec_idle;
static uint64_t tic_storage;
static void tic() { tic_storage = usecTimestamp(); }
static uint32_t toc() { return (uint32_t) (usecTimestamp() - tic_storage); }

// statistics
static float error_dist; // accumulated euclidean distance error
static float flighttime; // total flight time in seconds
static struct vec error_pos; // accumulated error in x/y/z

struct vec point2vec(point_t p)
{
  return mkvec(p.x, p.y, p.z);
}

void stabilizerInit(void)
{
  if(isInit)
    return;

  initUsecTimer();
  sensorsInit();
  stateEstimatorInit();
  // stateControllerInit();
  powerDistributionInit();
#if defined(SITAW_ENABLED)
  sitAwInit();
#endif
  positionExternalInit();
  trajectoryInit();

  xTaskCreate(stabilizerTask, STABILIZER_TASK_NAME,
              STABILIZER_TASK_STACKSIZE, NULL, STABILIZER_TASK_PRI, NULL);

  isInit = true;

  error_dist = 0;
  flighttime = 0;
  error_pos = vzero();
}

bool stabilizerTest(void)
{
  bool pass = true;

  pass &= sensorsTest();
  pass &= stateEstimatorTest();
  // pass &= stateControllerTest();
  pass &= powerDistributionTest();
  pass &= positionExternalTest();
  pass &= trajectoryTest();

  return pass;
}

state_t const *stabilizerState()
{
  return &state;
}

/* The stabilizer loop runs at 1kHz. It is the responsability or the different
 * functions to run slower by skipping call (ie. returning without modifying
 * the output structure).
 */
static void stabilizerTask(void* param)
{
  uint32_t tick = 0;
  uint32_t lastWakeTime;

  vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);

  //Wait for the system to be fully started to start stabilization loop
  systemWaitStart();

  // Wait for sensors to be calibrated
  lastWakeTime = xTaskGetTickCount();
  while(!sensorsAreCalibrated()) {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
  }

  // TODO before refactoring, this task used to wait
  // for a Vicon reading to initialize the EKF before entering its main loop.
  // Now the EKF does lazy-initialization. Do we still need to care about it here?

  while(1) {
    tic();
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
    int usec_idle_tmp = toc();

    sensorsAcquire(&sensorData, tick);

    tic();
    stateEstimator(&state, &sensorData, tick);
    if (RATE_DO_EXECUTE(RATE_500_HZ, tick)) {
      usec_ekf = toc();
    }

    // TODO: Disabled for now to avoid any side-effects
    //       Check if we can enable this again.
    // sitAwUpdateSetpoint(&setpoint, &sensorData, &state);

    // stateController(&control, &sensorData, &state, &setpoint, tick);
    if (RATE_DO_EXECUTE(RATE_500_HZ, tick)) {

      tic();
      commanderGetSetpoint(&setpoint, &state);
      usec_traj = toc();

      // Rate-controled YAW is moving YAW angle setpoint
      if (setpoint.mode.yaw == modeVelocity) {
         setpoint.attitude.yaw -= setpoint.attitudeRate.yaw/500.0;
        while (setpoint.attitude.yaw > 180.0)
          setpoint.attitude.yaw -= 360.0;
        while (setpoint.attitude.yaw < -180.0)
          setpoint.attitude.yaw+= 360.0;
      }

      if (!setpoint.enablePosCtrl) {
        setpoint.attitudeRate.yaw = 0;
      }

      tic();
      float const dt = 1.0f / RATE_500_HZ;
      positionControllerMellinger(&control, &state, &setpoint, dt);
      usec_ctrl = toc();

      if (!setpoint.enablePosCtrl) {
        control.thrust = setpoint.thrust;
      }

      m_roll = control.roll;
      m_pitch = control.pitch;
      m_yaw = control.yaw;

      bool upsideDown = sensorData.acc.z < -0.5;

      if (control.thrust == 0
          || upsideDown
          || ( setpoint.enablePosCtrl &&
             ( !sensorData.valid || trajectoryIsStopped())))
      {
        control.thrust = 0;
        control.roll = 0;
        control.pitch = 0;
        control.yaw = 0;

        // attitudeControllerResetAllPID();
        positionControllerReset();

        trajectoryStop();
        setpoint.attitude.yaw = state.attitude.yaw;
      } else if (trajectoryIsFlying()) {
          struct vec dist = vsub(point2vec(setpoint.position), point2vec(state.position));
          error_pos = vadd(error_pos, vscl(dt, vabs(dist)));
          error_dist += dt * vmag(dist);
          flighttime += dt;
      }
    }
    else {
      // since we do control every other loop, iterations where
      // we *don't* do control will have a shorter sleep time
      // because the previous iteration did the extra work
      usec_idle = usec_idle_tmp;
    }
    powerDistribution(&control);

#ifdef USD_LOG_DATA
    {
      UsdLogStruct logData;
      logData.timestamp =  xTaskGetTickCount();
      logData.gx = sensorData.gyro.x;
      logData.gy = sensorData.gyro.y;
      logData.gz = sensorData.gyro.z;
      logData.ax = sensorData.acc.x;
      logData.ay = sensorData.acc.y;
      logData.az = sensorData.acc.z;
      logData.mx = sensorData.mag.x;
      logData.my = sensorData.mag.y;
      logData.mz = sensorData.mag.z;
      usdQueueLogData(&logData);
    }
#endif

    tick++;
  }
}

LOG_GROUP_START(ctrltarget)
LOG_ADD(LOG_FLOAT, roll, &setpoint.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &setpoint.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &setpoint.attitude.yaw)
LOG_ADD(LOG_FLOAT, w_roll, &setpoint.attitudeRate.roll)
LOG_ADD(LOG_FLOAT, w_pitch, &setpoint.attitudeRate.pitch)
LOG_ADD(LOG_FLOAT, w_yaw, &setpoint.attitudeRate.yaw)
LOG_ADD(LOG_FLOAT, x, &setpoint.position.x)
LOG_ADD(LOG_FLOAT, y, &setpoint.position.y)
LOG_ADD(LOG_FLOAT, z, &setpoint.position.z)
LOG_ADD(LOG_FLOAT, v_x, &setpoint.velocity.x)
LOG_ADD(LOG_FLOAT, v_y, &setpoint.velocity.y)
LOG_ADD(LOG_FLOAT, v_z, &setpoint.velocity.z)
LOG_GROUP_STOP(ctrltarget)

LOG_GROUP_START(stabilizer)
LOG_ADD(LOG_FLOAT, roll, &state.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &state.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &state.attitude.yaw)
LOG_ADD(LOG_FLOAT, w_roll, &state.attitudeRate.roll)
LOG_ADD(LOG_FLOAT, w_pitch, &state.attitudeRate.pitch)
LOG_ADD(LOG_FLOAT, w_yaw, &state.attitudeRate.yaw)
LOG_ADD(LOG_FLOAT, roll_rad, &roll_rad)
LOG_ADD(LOG_FLOAT, pitch_rad, &pitch_rad)
LOG_ADD(LOG_FLOAT, yaw_rad, &yaw_rad)
LOG_ADD(LOG_UINT16, thrust, &control.thrust)
LOG_ADD(LOG_FLOAT, x, &state.position.x)
LOG_ADD(LOG_FLOAT, y, &state.position.y)
LOG_ADD(LOG_FLOAT, z, &state.position.z)
LOG_ADD(LOG_FLOAT, v_x, &state.velocity.x)
LOG_ADD(LOG_FLOAT, v_y, &state.velocity.y)
LOG_ADD(LOG_FLOAT, v_z, &state.velocity.z)
LOG_GROUP_STOP(stabilizer)

LOG_GROUP_START(acc)
LOG_ADD(LOG_FLOAT, x, &sensorData.acc.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.acc.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.acc.z)
LOG_GROUP_STOP(acc)


LOG_GROUP_START(extPosEst)
LOG_ADD(LOG_FLOAT, x, &sensorData.position.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.position.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.position.z)
LOG_GROUP_STOP(extPosEst)

LOG_GROUP_START(baro)
LOG_ADD(LOG_FLOAT, asl, &sensorData.baro.asl)
LOG_ADD(LOG_FLOAT, temp, &sensorData.baro.temperature)
LOG_ADD(LOG_FLOAT, pressure, &sensorData.baro.pressure)
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

LOG_GROUP_START(controller)
LOG_ADD(LOG_INT16, ctr_yaw, &control.yaw)
LOG_GROUP_STOP(controller)

LOG_GROUP_START(mot)
LOG_ADD(LOG_FLOAT, m_pitch, &m_pitch)
LOG_ADD(LOG_FLOAT, m_roll, &m_roll)
LOG_ADD(LOG_FLOAT, m_yaw, &m_yaw)
LOG_GROUP_STOP(mot)

LOG_GROUP_START(profiling)
LOG_ADD(LOG_UINT32, usec_ekf, &usec_ekf)
LOG_ADD(LOG_UINT32, usec_traj, &usec_traj)
LOG_ADD(LOG_UINT32, usec_ctrl, &usec_ctrl)
LOG_ADD(LOG_UINT32, usec_idle, &usec_idle)
LOG_GROUP_STOP(profiling)

LOG_GROUP_START(ctrlStat)
LOG_ADD(LOG_FLOAT, t, &flighttime)
LOG_ADD(LOG_FLOAT, edist, &error_dist)
LOG_ADD(LOG_FLOAT, ex, &error_pos.x)
LOG_ADD(LOG_FLOAT, ey, &error_pos.y)
LOG_ADD(LOG_FLOAT, ez, &error_pos.z)
LOG_GROUP_STOP(ctrlStat)
