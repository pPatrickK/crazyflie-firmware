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
#include "num.h"
#include "configblock.h"

//#include "console.h"
//#include "cfassert.h"

// Private types
typedef uint16_t fp16_t;
struct data {
  uint8_t startId;
  uint8_t count; // 1 - 3
  struct {
    fp16_t x; // m
    fp16_t y; // m
    fp16_t z; // m
    fp16_t yaw; // deg
  } position[3];
} __attribute__((packed));

// Global variables
static bool isInit = false;
static float lastX;
static float lastY;
static float lastZ;
static float lastYaw;
static uint64_t lastTime = 0;
static uint8_t my_id;

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
  uint64_t address = configblockGetRadioAddress();
  my_id = address & 0xFF;
  DEBUG_PRINT("posext. initialized: %d\n", my_id);
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
  // DEBUG_PRINT("lD: %d,%d\n", d->startId, d->count);
  // uint8_t my_id = 1;
  if (d->startId <= my_id && d->startId + d->count > my_id) {
    uint8_t i = my_id - d->startId;
    lastX = half2single(d->position[i].x);
    lastY = half2single(d->position[i].y);
    lastZ = half2single(d->position[i].z);
    lastYaw = half2single(d->position[i].yaw);

    // uint16_t dt = xTaskGetTickCount() - lastTime;
    // DEBUG_PRINT("dt: %d\n", dt);
    // DEBUG_PRINT("lastData: %f,%f,%f,%f,%d\n", lastX, lastY, lastZ, lastYaw, dt);

    lastTime = xTaskGetTickCount();
  }
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
