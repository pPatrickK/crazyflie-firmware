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

struct data {
  float x; //m
  float y; //m
  float z; //m
  float yaw; //deg
} __attribute__((packed));

// Global variables
static bool isInit = false;
static float lastX;
static float lastY;
static float lastZ;
static float lastYaw;
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
  float* yaw,
  uint16_t* last_time_in_ms)
{
  *x = lastX;
  *y = lastY;
  *z = lastZ;
  *yaw = lastYaw;
  if (xTaskGetTickCount() - lastTime < 10 * 1000) {
    *last_time_in_ms = xTaskGetTickCount() - lastTime;
  } else {
    *last_time_in_ms = 10 * 1000;
  }
}

static void positionExternalCrtpCB(CRTPPacket* pk)
{
  struct data* d = ((struct data*)pk->data);
  lastX = d->x;
  lastY = d->y;
  lastZ = d->z;
  lastYaw = d->yaw;
  lastTime = xTaskGetTickCount();
}
