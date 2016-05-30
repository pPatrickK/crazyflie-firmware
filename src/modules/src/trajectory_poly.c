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
 * trajectory.c: Module to receive trajectory from the host
 */


#include <string.h>
#include <errno.h>
#include <math.h>
// #include <stdint.h>
// #include <stdbool.h>

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"
// #include "timers.h"
// #include "semphr.h"

// #include "config.h"
#include "crtp.h"
#include "trajectory.h"
#include "debug.h"
#include "position_external.h"
#include "pptraj.h"
#include "usec_time.h"

//#include "console.h"
//#include "cfassert.h"

// Private types
enum TrajectoryCommand_e {
  COMMAND_RESET   = 0,
  COMMAND_ADD     = 1,
  COMMAND_START   = 2,
  //COMMAND_STATE = 3,
  COMMAND_TAKEOFF = 4,
  COMMAND_LAND    = 5,
};

// struct data_reset {
// } __attribute__((packed));


struct data_add {
  uint8_t id;
  uint8_t offset;
  uint8_t size;
  float values[7];
} __attribute__((packed));
/*
struct data_state {
  uint8_t state;
};

struct data_takeoff {
  float height; // m (absolute)
  uint16_t time_from_start; // ms
};

struct data_land {
  float height; // m (absolute)
  uint16_t time_from_start; // ms
};

// enum state_e {
//   STATE_IDLE       = 0,
//   STATE_TAKING_OFF = 1,
//   STATE_LANDING    = 2,
//   STATE_FLYING     = 3,
// };

// static state_e state = STATE_IDLE;

#define MAX_TRAJECTORY_ENTRIES 100

struct trajectoryEntry {
  uint16_t time_from_start;
  trajectoryPoint_t point;
};
*/

// Global variables
static bool isInit = false;
static CRTPPacket p;
static trajectoryState_t state = TRAJECTORY_STATE_IDLE;
static struct piecewise_traj pp;

// Private functions
static void trajectoryTask(void * prm);
static int trajectoryReset(void);
static int trajectoryAdd(const struct data_add* data);
static int trajectoryStart(void);
// static int trajectoryState(const struct data_state* data);
static int trajectoryTakeoff();
static int trajectoryLand();


void trajectoryInit(void)
{
  if(isInit) {
    return;
  }

  //Start the trajectory task
  xTaskCreate(trajectoryTask, TRAJECTORY_TASK_NAME,
              TRAJECTORY_TASK_STACKSIZE, NULL, TRAJECTORY_TASK_PRI, NULL);

  initUsecTimer(); // TODO: Move somewhere "central"
  isInit = true;
  DEBUG_PRINT("traj. initialized.\n");
}

bool trajectoryTest(void)
{
  return isInit;
}

void pp_eval_to_trajectory_point(struct traj_eval const *ev, trajectoryPoint_t *goal)
{
    goal->x = ev->pos.x;
    goal->y = ev->pos.y;
    goal->z = ev->pos.z;
    goal->velocity_x = ev->vel.x;
    goal->velocity_y = ev->vel.y;
    goal->velocity_z = ev->vel.z;
	goal->yaw = ev->yaw;
	goal->omega = ev->omega;
}

void trajectoryGetCurrentGoal(trajectoryPoint_t* goal)
{
  float t = usecTimestamp() / 1e6; //xTaskGetTickCount() / 1000.0; // TODO not magic number
  struct traj_eval ev = piecewise_eval(&pp, t, 0.03); //  TODO mass not magic number
  pp_eval_to_trajectory_point(&ev, goal);

  if (piecewise_is_finished(&pp)) {
    if (state == TRAJECTORY_STATE_TAKING_OFF) {
      state = TRAJECTORY_STATE_FLYING;
    }
    if (state == TRAJECTORY_STATE_LANDING) {
      state = TRAJECTORY_STATE_IDLE;
    }
  }
}

// void trajectoryIsIdle()
// {
//   return state == STATE_IDLE;
// }

// void trajectorySetIdle()
// {
//   state = STATE_IDLE;
// }

void trajectoryGetState(trajectoryState_t* st)
{
  *st = state;
}

void trajectorySetState(trajectoryState_t st)
{
  state = st;
}

void trajectoryTask(void * prm)
{
  int ret;
  crtpInitTaskQueue(CRTP_PORT_TRAJECTORY);

  while(1) {
    crtpReceivePacketBlock(CRTP_PORT_TRAJECTORY, &p);
    // DEBUG_PRINT("Recv. sth.\n");

    switch(p.data[0])
    {
      case COMMAND_RESET:
        ret = trajectoryReset();
        break;
      case COMMAND_ADD:
        ret = trajectoryAdd((const struct data_add*)&p.data[1]);
        break;
      case COMMAND_START:
        ret = trajectoryStart();
        break;
      // case COMMAND_STATE:
      //   ret = trajectoryState((const struct data_state*)&p.data[1]);
      case COMMAND_TAKEOFF:
        ret = trajectoryTakeoff();
        break;
      case COMMAND_LAND:
        ret = trajectoryLand();
        break;
      default:
        ret = ENOEXEC;
        break;
    }

    //answer
    p.data[2] = ret;
    p.size = 3;
    crtpSendPacket(&p);
  }
}


int trajectoryReset(void)
{
  pp.n_pieces = 0;
  DEBUG_PRINT("trajectoryReset\n");

  return 0;
}

int trajectoryAdd(const struct data_add* data)
{
  if (data->id < PP_MAX_PIECES
      && data->offset + data-> size < sizeof(pp.pieces[pp.n_pieces])) {
    uint8_t size = data->size;
    uint8_t* ptr = (uint8_t*)&(data->values[0]);
    if (data->offset == 0) {
      pp.pieces[pp.n_pieces].duration = data->values[0];
      size -= 4;
      ptr += 4;
    }
    memcpy(((uint8_t*)pp.pieces[pp.n_pieces].p) + data->offset, ptr, size);
    pp.n_pieces = data->id + 1;
    DEBUG_PRINT("trajectoryAdd: %d\n", data->id);
    return 0;
  }

  return ENOMEM;
}


int trajectoryStart(void)
{
  pp.t_begin_piece = usecTimestamp() / 1e6;
  pp.cursor = 0;
  return 0;
}

// int trajectoryState(const struct data_state* data)
// {
//   state = data->state;
//   return 0;
// }


// for takeoff/landing traj, set x, y, and yaw to current position
static void set_xyyaw_current(struct poly4d *p)
{
  float x, y, z, q0, q1, q2, q3;
  uint16_t last_time_in_ms;
  positionExternalGetLastData(&x, &y, &z, &q0, &q1, &q2, &q3, &last_time_in_ms);
  float yaw = quat2rpy(mkquat(q0, q1, q2, q3)).z;
  poly4d_shift(p, x, y, 0, yaw);
}

int trajectoryTakeoff()
{
  if (state != TRAJECTORY_STATE_IDLE) {
    return 1;
  }

  pp.pieces[0] = poly4d_takeoff;
  set_xyyaw_current(&pp.pieces[0]);
  pp.n_pieces = 1;

  state = TRAJECTORY_STATE_TAKING_OFF;
  trajectoryStart();
  return 0;
}

int trajectoryLand()
{
  if (state != TRAJECTORY_STATE_FLYING) {
    return 1;
  }

  pp.pieces[0] = poly4d_takeoff;
  poly4d_scale(&pp.pieces[0], 0, 0, -0.97, 0);
  poly4d_shift(&pp.pieces[0], 0, 0, 1, 0);
  set_xyyaw_current(&pp.pieces[0]);
  pp.n_pieces = 1;

  state = TRAJECTORY_STATE_LANDING;
  trajectoryStart();
  return 0;
}
