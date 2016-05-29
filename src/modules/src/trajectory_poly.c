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


// #include <string.h>
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
#include "position_external_bringup.h"
#include "pptraj.h"

//#include "console.h"
//#include "cfassert.h"

// Private types
enum TrajectoryCommand_e {
  //COMMAND_RESET   = 0,
  //COMMAND_ADD     = 1,
  //COMMAND_START   = 2,
  //COMMAND_STATE = 3,
  COMMAND_TAKEOFF = 4,
  COMMAND_LAND    = 5,
};

// struct data_reset {
// } __attribute__((packed));

/*
struct data_add {
  uint8_t id;
  uint16_t time_from_start; // ms; 0 marks the beginning of the execution
  float x; // m
  float y; // m
  float z; // m
  float velocity_x; // m/s
  float velocity_y; // m/s
  float velocity_z; // m/s
  int16_t yaw; // rad * 1000
} __attribute__((packed));

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
//static struct trajectoryEntry trajectory[MAX_TRAJECTORY_ENTRIES];
//static uint16_t numEntries = 0;
//static uint16_t currentEntry = 0;
//static struct trajectoryEntry lastValidTrajectoryEntry;
static uint64_t startTime = 0;
static trajectoryState_t state = TRAJECTORY_STATE_IDLE;

//Private functions
static void trajectoryTask(void * prm);
//static int trajectoryReset(void);
//static int trajectoryAdd(const struct data_add* data);
//static int trajectoryStart(void);
// static int trajectoryState(const struct data_state* data);
static int trajectoryTakeoff();
static int trajectoryLand();

static struct poly4d poly;


void trajectoryInit(void)
{
  if(isInit) {
    return;
  }

  //Start the trajectory task
  xTaskCreate(trajectoryTask, TRAJECTORY_TASK_NAME,
              TRAJECTORY_TASK_STACKSIZE, NULL, TRAJECTORY_TASK_PRI, NULL);

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
  uint64_t t_tick = xTaskGetTickCount();
  float t = ((float)(t_tick - startTime)) / 1000.0; // TODO not magic number

  if (t < poly.duration) {
    struct traj_eval ev = poly4d_eval(&poly, t, 0.03); //  TODO mass not magic number
	pp_eval_to_trajectory_point(&ev, goal);
  } else {
    struct traj_eval ev = poly4d_eval(&poly, poly.duration, 0.03);//  TODO mass not magic number
	pp_eval_to_trajectory_point(&ev, goal);

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
      //case COMMAND_RESET:
        //ret = trajectoryReset();
        //break;
      //case COMMAND_ADD:
        //ret = trajectoryAdd((const struct data_add*)&p.data[1]);
        //break;
      //case COMMAND_START:
        //ret = trajectoryStart();
        //break;
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

/*
int trajectoryReset(void)
{
  numEntries = 0;
  DEBUG_PRINT("trajectoryReset\n");

  return 0;
}

int trajectoryAdd(const struct data_add* data)
{
  if (data->id < MAX_TRAJECTORY_ENTRIES) {
    trajectory[numEntries].time_from_start = data->time_from_start;
    trajectory[numEntries].point.x = data->x;
    trajectory[numEntries].point.y = data->y;
    trajectory[numEntries].point.z = data->z;
    trajectory[numEntries].point.velocity_x = data->velocity_x;
    trajectory[numEntries].point.velocity_y = data->velocity_y;
    trajectory[numEntries].point.velocity_z = data->velocity_z;
    trajectory[numEntries].point.yaw = data->yaw / 1000.0;
    numEntries = data->id + 1;
    DEBUG_PRINT("trajectoryAdd: %d\n", data->id);
    return 0;
  }

  return ENOMEM;
}
*/

int trajectoryStart(void)
{
  startTime = xTaskGetTickCount();
  return 0;
}

// int trajectoryState(const struct data_state* data)
// {
//   state = data->state;
//   return 0;
// }


// set x, y, and yaw to current position, hold constant
static void set_xyyaw_constant(struct poly4d *p)
{
  float x, y, z, q0, q1, q2, q3;
  uint16_t last_time_in_ms;
  positionExternalBringupGetLastData(&x, &y, &z, &q0, &q1, &q2, &q3, &last_time_in_ms);
  float yaw = quat2rpy(mkquat(q0, q1, q2, q3)).z;
  poly4d_set_constant(p, 0, x);
  poly4d_set_constant(p, 1, y);
  poly4d_set_constant(p, 3, yaw);
}

int trajectoryTakeoff()
{
  if (state != TRAJECTORY_STATE_IDLE) {
    return 1;
  }
  
  poly = poly4d_takeoff;
  set_xyyaw_constant(&poly);

  state = TRAJECTORY_STATE_TAKING_OFF;
  trajectoryStart();
  return 0;
}

int trajectoryLand()
{
  if (state != TRAJECTORY_STATE_FLYING) {
    return 1;
  }

  poly = poly4d_landing;
  set_xyyaw_constant(&poly);

  state = TRAJECTORY_STATE_LANDING;
  trajectoryStart();
  return 0;
}
