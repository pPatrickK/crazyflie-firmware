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


// #include <string.h>
#include <errno.h>
// #include <stdint.h>
// #include <stdbool.h>

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"
// #include "timers.h"
// #include "semphr.h"

// #include "config.h"
#include "crtp.h"
#include "position_external.h"
#include "debug.h"

//#include "console.h"
//#include "cfassert.h"

// Private types
struct data {
  float x;
  float y;
  float z;
  float yaw;
} __attribute__((packed));

// Global variables
static bool isInit = false;
// static CRTPPacket p;
static struct data lastData;
static uint64_t lastTime = 0;

//Private functions
// static void positionExternalTask(void * prm);
static void positionExternalCrtpCB(CRTPPacket* pk);

void positionExternalInit(void)
{
  if(isInit) {
    return;
  }

  crtpInit();
  crtpRegisterPortCB(CRTP_PORT_POSEXT, positionExternalCrtpCB);

  //Start the positionExternal task
  // xTaskCreate(positionExternalTask, POSEXT_TASK_NAME,
  //             POSEXT_TASK_STACKSIZE, NULL, POSEXT_TASK_PRI, NULL);

  isInit = true;
  DEBUG_PRINT("posext. initialized.\n");
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
  *x = lastData.x;
  *y = lastData.y;
  *z = lastData.z;
  *yaw = lastData.yaw;
  if (xTaskGetTickCount() - lastTime < 10 * 1000) {
    *last_time_in_ms = xTaskGetTickCount() - lastTime;
  } else {
    *last_time_in_ms = 10 * 1000;
  }
}

static void positionExternalCrtpCB(CRTPPacket* pk)
{
  lastData = *((struct data*)pk->data);
  lastTime = xTaskGetTickCount();
}

// void positionExternalTask(void * prm)
// {
//   crtpInitTaskQueue(CRTP_PORT_POSEXT);

//   while(1) {
//     crtpReceivePacketBlock(CRTP_PORT_POSEXT, &p);
//     lastData = *((struct data*)&p.data[1]);
//     lastTime = xTaskGetTickCount();
//     DEBUG_PRINT("lastData: %f,%f,%f,%f\n", lastData.x, lastData.y, lastData.z, lastData.yaw);

//     //answer
//     // p.data[2] = ret;
//     p.size = 1;
//     crtpSendPacket(&p);
//   }
// }
