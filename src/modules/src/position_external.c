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


static bool isInit = false;
static uint8_t my_id;

struct position_state
{
  struct vec pos;
  struct quat quat;
  struct vec velocity; // for logging only
  struct vec rpy;      // for logging only
  uint32_t timestamp_ms;
  uint16_t dt;
};

bool positionExternalFresh = false;
static struct position_state external;
// stores the position from messages with id = GLOBAL_ID.
// these are meant to be used by special interactive planners.
bool positionInteractiveFresh = false;
static struct position_state interactive;
positionInteractiveCallback interactiveCallback = NULL;
void positionInteractiveSetCallback(positionInteractiveCallback cb)
{
  interactiveCallback = cb;
}


//Private functions
static void positionExternalCrtpCB(CRTPPacket* pk);

void positionExternalInit(void)
{
  if (isInit) {
    return;
  }

  crtpInit();
  crtpRegisterPortCB(CRTP_PORT_POSEXT, positionExternalCrtpCB);

  uint64_t address = configblockGetRadioAddress();
  my_id = address & 0xFF;
  DEBUG_PRINT("posextbrinup. initialized: %d\n", my_id);

  external.timestamp_ms = 0;
  interactive.timestamp_ms = 0;

  isInit = true;
}

bool positionExternalTest(void)
{
  return isInit;
}

static void update(struct data_vicon const *d, int i, struct position_state *state)
{
  float x = position_fix2float(d->pose[i].x);
  float y = position_fix2float(d->pose[i].y);
  float z = position_fix2float(d->pose[i].z);
  struct vec pos = mkvec(x, y, z);

  uint32_t tick = xTaskGetTickCount();

  // logging-only stuff, not used by rest of firmware
  if (state->timestamp_ms != 0) {
    float dt_sec = (tick - state->timestamp_ms) / 1000.0f;
    state->velocity = vdiv(vsub(pos, state->pos), dt_sec);
  }
  state->rpy = vscl(180 / M_PI, quat2rpy(state->quat));

  state->pos = pos;
  float q[4];
  quatdecompress(d->pose[i].quat, q);
  state->quat = qloadf(q);

  if ((tick - state->timestamp_ms) > 10 * 1000) {
    state->dt = 10 * 1000;
  } else {
    state->dt = tick - state->timestamp_ms;
  }
  state->timestamp_ms = tick;
}

static void read(struct position_state *state, struct position_message *msg)
{
  msg->pos = state->pos;
  msg->quat = state->quat;
  msg->timestamp_ms = state->timestamp_ms;
}

void positionExternalGetLastData(struct position_message *msg)
{
  read(&external, msg);
}

void positionInteractiveGetLastData(struct position_message *msg)
{
  read(&interactive, msg);
}

static void positionExternalCrtpCB(CRTPPacket* pk)
{
  struct data_vicon* d = ((struct data_vicon*)pk->data);
  for (int i=0; i < 2; ++i) {
    if (d->pose[i].id == my_id) {
      update(d, i, &external);
      positionExternalFresh = true;
    }
    else if (d->pose[i].id == INTERACTIVE_ID) {
      update(d, i, &interactive);
      positionInteractiveFresh = true;
      if (interactiveCallback != NULL) {
        struct position_message msg;
        read(&interactive, &msg);
        (*interactiveCallback)(&msg);
      }
    }
  }
}

LOG_GROUP_START(vicon)
LOG_ADD(LOG_FLOAT, x, &external.pos.x)
LOG_ADD(LOG_FLOAT, y, &external.pos.y)
LOG_ADD(LOG_FLOAT, z, &external.pos.z)
LOG_ADD(LOG_FLOAT, v_x, &external.velocity.x)
LOG_ADD(LOG_FLOAT, v_y, &external.velocity.y)
LOG_ADD(LOG_FLOAT, v_z, &external.velocity.z)
LOG_ADD(LOG_FLOAT, roll, &external.rpy.x)
LOG_ADD(LOG_FLOAT, pitch, &external.rpy.y)
LOG_ADD(LOG_FLOAT, yaw, &external.rpy.z)
LOG_ADD(LOG_INT16, dt, &external.dt)
LOG_GROUP_STOP(vicon)
