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
#include "commander.h"
#include "crtp_localization_service.h"
#include "sitaw.h"
#include "controller.h"
#include "power_distribution.h"

#include "estimator_kalman.h"
#include "estimator.h"
#include "crtp_commander_high_level.h"

static bool isInit;
static bool emergencyStop = false;
static bool upsideDown = false; // added by patrick for debugging
static float aZimu; // added by patrick for debugging
static uint32_t usdCnt = 0; // added by patrick; quick fix test of upsideDown Bug
static int emergencyStopTimeout = EMERGENCY_STOP_TIMEOUT_DISABLED;

// State variables for the stabilizer
static setpoint_t setpoint;
static sensorData_t sensorData;
static state_t state;
static control_t control;

// statistics
static float error_dist; // euclidean distance error
static float error_dist_last; // euclidean distance error over the last 1000 ms

struct vec point2vec(point_t p)
{
  return mkvec(p.x, p.y, p.z);
}

static StateEstimatorType estimatorType;
static ControllerType controllerType;

static void stabilizerTask(void* param);

void stabilizerInit(StateEstimatorType estimator)
{
  if(isInit)
    return;

  sensorsInit();
  stateEstimatorInit(estimator);
  controllerInit(ControllerTypeAny);
  powerDistributionInit();
  if (estimator == kalmanEstimator)
  {
    sitAwInit();
  }
  estimatorType = getStateEstimator();
  controllerType = getControllerType();

  xTaskCreate(stabilizerTask, STABILIZER_TASK_NAME,
              STABILIZER_TASK_STACKSIZE, NULL, STABILIZER_TASK_PRI, NULL);

  error_dist = 0;
  error_dist_last = 0;
  isInit = true;
}

bool stabilizerTest(void)
{
  bool pass = true;

  pass &= sensorsTest();
  pass &= stateEstimatorTest();
  pass &= controllerTest();
  pass &= powerDistributionTest();

  return pass;
}

static void checkEmergencyStopTimeout()
{
  if (emergencyStopTimeout >= 0) {
    emergencyStopTimeout -= 1;

    if (emergencyStopTimeout == 0) {
      emergencyStop = true;
    }
  }
}

/* The stabilizer loop runs at 1kHz (stock) or 500Hz (kalman). It is the
 * responsibility of the different functions to run slower by skipping call
 * (ie. returning without modifying the output structure).
 */

static void stabilizerTask(void* param)
{
  uint32_t tick;
  uint32_t lastWakeTime;
  vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);

  //Wait for the system to be fully started to start stabilization loop
  systemWaitStart();

  // Wait for sensors to be calibrated
  lastWakeTime = xTaskGetTickCount ();
  while(!sensorsAreCalibrated()) {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
  }
  // Initialize tick to something else then 0
  tick = 1;

  while(1) {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));

    // allow to update estimator dynamically
    if (getStateEstimator() != estimatorType) {
      stateEstimatorInit(estimatorType);
      estimatorType = getStateEstimator();
    }
    // allow to update controller dynamically
    if (getControllerType() != controllerType) {
      controllerInit(controllerType);
      controllerType = getControllerType();
    }

    getExtPosition(&state);
    stateEstimator(&state, &sensorData, &control, tick);

    commanderGetSetpoint(&setpoint, &state);

    sitAwUpdateSetpoint(&setpoint, &sensorData, &state);

    controller(&control, &setpoint, &sensorData, &state, tick);

    checkEmergencyStopTimeout(); // is currently disabled

    // TODO: this should go into the sitAw framework
    aZimu = sensorData.acc.z; // added by patrick
    //upsideDown = sensorData.acc.z < -0.5f; // done by USC
    if(sensorData.acc.z < -0.5f){ // quick fix for upsideDown Bug; (too sensible)
      usdCnt += 1;
    }
    else{
      usdCnt = 0;
    }
    if(usdCnt==500){
      upsideDown = 1; // true
    }

    if (emergencyStop || upsideDown) { // something kills also the log
      powerStop();
      controllerInit(getControllerType());
      crtpCommanderHighLevelStop();
    } else {
      powerDistribution(&control);
    }

    // stats
    if (!crtpCommanderHighLevelIsStopped()) {
      float const dt = 1.0f / RATE_MAIN_LOOP;
      struct vec dist = vsub(point2vec(setpoint.position), point2vec(state.position));
      error_dist += dt * vmag(dist);

      if (tick % 1000 == 0) {
        error_dist_last = error_dist;
        error_dist = 0;
      }
    }

    tick++;
  }
}

void stabilizerSetEmergencyStop()
{
  emergencyStop = true;
}

void stabilizerResetEmergencyStop()
{
  emergencyStop = false;
}

void stabilizerSetEmergencyStopTimeout(int timeout)
{
  emergencyStop = false;
  emergencyStopTimeout = timeout;
}

PARAM_GROUP_START(stabilizer)
PARAM_ADD(PARAM_UINT8, estimator, &estimatorType)
PARAM_ADD(PARAM_UINT8, controller, &controllerType)
PARAM_GROUP_STOP(stabilizer)

LOG_GROUP_START(ctrltarget)
LOG_ADD(LOG_FLOAT, aZimu, &aZimu)
LOG_ADD(LOG_UINT8, upsideDown, &upsideDown)
LOG_ADD(LOG_UINT8, emergencyStop, &emergencyStop)
LOG_ADD(LOG_UINT8, usdCnt, &usdCnt)
//LOG_ADD(LOG_FLOAT, x, &setpoint.position.x)
//LOG_ADD(LOG_FLOAT, y, &setpoint.position.y)
//LOG_ADD(LOG_FLOAT, z, &setpoint.position.z)

//LOG_ADD(LOG_FLOAT, vx, &setpoint.velocity.x)
//LOG_ADD(LOG_FLOAT, vy, &setpoint.velocity.y)
//LOG_ADD(LOG_FLOAT, vz, &setpoint.velocity.z)

LOG_ADD(LOG_FLOAT, ax, &setpoint.acceleration.x)
LOG_ADD(LOG_FLOAT, ay, &setpoint.acceleration.y)
LOG_ADD(LOG_FLOAT, az, &setpoint.acceleration.z)

LOG_ADD(LOG_FLOAT, roll, &setpoint.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &setpoint.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &setpoint.attitude.yaw)

LOG_ADD(LOG_FLOAT, rollRate, &setpoint.attitudeRate.roll)
LOG_ADD(LOG_FLOAT, pitchRate, &setpoint.attitudeRate.pitch)
LOG_ADD(LOG_FLOAT, yawRate, &setpoint.attitudeRate.yaw)
LOG_GROUP_STOP(ctrltarget)

// LOG_GROUP_START(stabilizer)
// LOG_ADD(LOG_FLOAT, roll, &state.attitude.roll)
// LOG_ADD(LOG_FLOAT, pitch, &state.attitude.pitch)
// LOG_ADD(LOG_FLOAT, yaw, &state.attitude.yaw)
// LOG_ADD(LOG_UINT16, thrust, &control.thrust)
// LOG_GROUP_STOP(stabilizer)

// LOG_GROUP_START(acc)
// LOG_ADD(LOG_FLOAT, x, &sensorData.acc.x)
// LOG_ADD(LOG_FLOAT, y, &sensorData.acc.y)
// LOG_ADD(LOG_FLOAT, z, &sensorData.acc.z)
// LOG_GROUP_STOP(acc)

#ifdef LOG_SEC_IMU
LOG_GROUP_START(accSec)
LOG_ADD(LOG_FLOAT, x, &sensorData.accSec.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.accSec.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.accSec.z)
LOG_GROUP_STOP(accSec)
#endif

// LOG_GROUP_START(baro)
// LOG_ADD(LOG_FLOAT, asl, &sensorData.baro.asl)
// LOG_ADD(LOG_FLOAT, temp, &sensorData.baro.temperature)
// LOG_ADD(LOG_FLOAT, pressure, &sensorData.baro.pressure)
// LOG_GROUP_STOP(baro)

// LOG_GROUP_START(gyro)
// LOG_ADD(LOG_FLOAT, x, &sensorData.gyro.x)
// LOG_ADD(LOG_FLOAT, y, &sensorData.gyro.y)
// LOG_ADD(LOG_FLOAT, z, &sensorData.gyro.z)
// LOG_GROUP_STOP(gyro)

#ifdef LOG_SEC_IMU
LOG_GROUP_START(gyroSec)
LOG_ADD(LOG_FLOAT, x, &sensorData.gyroSec.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.gyroSec.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.gyroSec.z)
LOG_GROUP_STOP(gyroSec)
#endif

// LOG_GROUP_START(mag)
// LOG_ADD(LOG_FLOAT, x, &sensorData.mag.x)
// LOG_ADD(LOG_FLOAT, y, &sensorData.mag.y)
// LOG_ADD(LOG_FLOAT, z, &sensorData.mag.z)
// LOG_GROUP_STOP(mag)

// LOG_GROUP_START(controller)
// LOG_ADD(LOG_INT16, ctr_yaw, &control.yaw)
// LOG_GROUP_STOP(controller)

LOG_GROUP_START(stateEstimate)
LOG_ADD(LOG_FLOAT, x, &state.position.x)
LOG_ADD(LOG_FLOAT, y, &state.position.y)
LOG_ADD(LOG_FLOAT, z, &state.position.z)

LOG_ADD(LOG_FLOAT, vx, &state.velocity.x)
LOG_ADD(LOG_FLOAT, vy, &state.velocity.y)
LOG_ADD(LOG_FLOAT, vz, &state.velocity.z)

LOG_ADD(LOG_FLOAT, ax, &state.acc.x)
LOG_ADD(LOG_FLOAT, ay, &state.acc.y)
LOG_ADD(LOG_FLOAT, az, &state.acc.z)

LOG_ADD(LOG_FLOAT, roll, &state.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &state.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &state.attitude.yaw)
LOG_GROUP_STOP(stateEstimate)

LOG_GROUP_START(ctrlStat)
LOG_ADD(LOG_FLOAT, edist, &error_dist_last)
LOG_GROUP_STOP(ctrlStat)
