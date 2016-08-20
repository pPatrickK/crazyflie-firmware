/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2012 BitCraze AB
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
 * position_external.c: Module to receive current position and yaw from external source
 */

#include <errno.h>

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"

#include "crtp.h"
#include "position_external.h"
#include "debug.h"
#include "num.h"
#include "configblock.h"
#include "log.h"
#include "math3d.h"
#include "packetdef.h"
#include "quatcompress.h"


// Global variables
bool positionExternalFresh = false;
static bool isInit = false;
static float lastX;
static float lastY;
static float lastZ;
static float lastQ0;
static float lastQ1;
static float lastQ2;
static float lastQ3;
static struct vec lastRPY;
static uint64_t lastTime = 0;
static uint8_t my_id;
static float v_x;
static float v_y;
static float v_z;
static uint16_t dt;

positionInteractiveCallback interactiveCallback = NULL;

//Private functions
static void positionExternalCrtpCB(CRTPPacket* pk);

void positionExternalInit(void)
{
  if(isInit) {
    return;
  }

  crtpInit();
  crtpRegisterPortCB(CRTP_PORT_POSEXT, positionExternalCrtpCB);

  isInit = true;

  uint64_t address = configblockGetRadioAddress();
  my_id = address & 0xFF;
  DEBUG_PRINT("posextbrinup. initialized: %d\n", my_id);
}

bool positionExternalTest(void)
{
  return isInit;
}

void positionExternalGetLastData(
  float* x,
  float* y,
  float* z,
  float* q0,
  float* q1,
  float* q2,
  float* q3,
  float* vx,
  float* vy,
  float* vz,
  uint16_t* last_time_in_ms)
{
  *x = lastX;
  *y = lastY;
  *z = lastZ;
  *q0 = lastQ0;
  *q1 = lastQ1;
  *q2 = lastQ2;
  *q3 = lastQ3;
  *vx = v_x;
  *vy = v_y;
  *vz = v_z;
  if (xTaskGetTickCount() - lastTime < 10 * 1000) {
    *last_time_in_ms = xTaskGetTickCount() - lastTime;
  } else {
    *last_time_in_ms = 10 * 1000;
  }
  dt = *last_time_in_ms;
}

void setPositionInteractiveCallback(positionInteractiveCallback cb)
{
  interactiveCallback = cb;
}

static void positionExternalCrtpCB(CRTPPacket* pk)
{
  struct data_vicon* d = ((struct data_vicon*)pk->data);
  for (int i=0; i < 2; ++i) {
    if (d->pose[i].id == my_id) {
      float x = position_fix24_to_float(d->pose[i].x);
      float y = position_fix24_to_float(d->pose[i].y);
      float z = position_fix24_to_float(d->pose[i].z);

      if (lastTime != 0) {
        float dt = (xTaskGetTickCount() - lastTime) / 1000.0f;
        dt = fmax(dt, 0.005);
        v_x = (x - lastX) / dt;
        v_y = (y - lastY) / dt;
        v_z = (z - lastZ) / dt;
      }

      lastX = x;
      lastY = y;
      lastZ = z;

      float q[4];
      quatdecompress(d->pose[i].quat, q);
      lastQ0 = q[0];
      lastQ1 = q[1];
      lastQ2 = q[2];
      lastQ3 = q[3];

      lastRPY = vscl(180 / M_PI, quat2rpy(mkquat(lastQ0, lastQ1, lastQ2, lastQ3)));

      lastTime = xTaskGetTickCount();
      positionExternalFresh = true;
    }
    else if (d->pose[i].id == INTERACTIVE_ID && interactiveCallback != NULL) {
      float x = position_fix24_to_float(d->pose[i].x);
      float y = position_fix24_to_float(d->pose[i].y);
      float z = position_fix24_to_float(d->pose[i].z);
      struct vec pos = mkvec(x, y, z);

      float q[4];
      quatdecompress(d->pose[i].quat, q);
      struct quat quat = qloadf(q);

      (*interactiveCallback)(&pos, &quat);
    }
  }
}

LOG_GROUP_START(vicon)
LOG_ADD(LOG_FLOAT, v_x, &v_x)
LOG_ADD(LOG_FLOAT, v_y, &v_y)
LOG_ADD(LOG_FLOAT, v_z, &v_z)
LOG_ADD(LOG_INT16, dt, &dt)
LOG_ADD(LOG_FLOAT, roll, &lastRPY.x)
LOG_ADD(LOG_FLOAT, pitch, &lastRPY.y)
LOG_ADD(LOG_FLOAT, yaw, &lastRPY.z)
LOG_ADD(LOG_FLOAT, x, &lastX)
LOG_ADD(LOG_FLOAT, y, &lastY)
LOG_ADD(LOG_FLOAT, z, &lastZ)
LOG_GROUP_STOP(vicon)
