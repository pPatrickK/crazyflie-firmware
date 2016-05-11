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

bool positionExternalFresh = false;

struct data {
  float x; //m
  float y; //m
  float z; //m
  float q0;
  float q1;
  float q2;
  float q3;
} __attribute__((packed));

// Global variables
static bool isInit = false;
static struct data lastData;
static uint64_t lastTime = 0;

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
  uint16_t* last_time_in_ms)
{
  *x = lastData.x;
  *y = lastData.y;
  *z = lastData.z;
  *q0 = lastData.q0;
  *q1 = lastData.q1;
  *q2 = lastData.q2;
  *q3 = lastData.q3;
  if (xTaskGetTickCount() - lastTime < 10 * 1000) {
    *last_time_in_ms = xTaskGetTickCount() - lastTime;
  } else {
    *last_time_in_ms = 10 * 1000;
  }
}

static void positionExternalCrtpCB(CRTPPacket* pk)
{
  struct data* d = ((struct data*)pk->data);
  lastData = *d;
  lastTime = xTaskGetTickCount();
  positionExternalFresh = true;
}
