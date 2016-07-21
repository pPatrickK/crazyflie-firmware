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
#include "packetdef.h"
#include "usec_time.h"

//#include "console.h"
//#include "cfassert.h"


// enum state_e {
//   STATE_IDLE       = 0,
//   STATE_TAKING_OFF = 1,
//   STATE_LANDING    = 2,
//   STATE_FLYING     = 3,
// };

// static state_e state = STATE_IDLE;
/*
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

static struct piecewise_traj* ppFront;
static struct piecewise_traj* ppBack;
static struct piecewise_traj pp1;
static struct piecewise_traj pp2;

static struct ellipse_traj ellipse;

// Private functions
static void trajectoryTask(void * prm);
static int trajectoryReset(void);
static int trajectoryAdd(const struct data_add* data);
static int trajectoryStart(void);
// static int trajectoryState(const struct data_state* data);
static int trajectoryTakeoff();
static int trajectoryLand();
static int trajectoryHover(const struct data_hover* data);
static int ellipseStart();

// static bool stretched = false;


void trajectoryInit(void)
{
  if(isInit) {
    return;
  }

  ppFront = &pp1;
  ppBack = &pp2;

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

// works in our structs, not public to rest of firmware
static struct traj_eval current_goal()
{
  float t = usecTimestamp() / 1e6;
  if (state == TRAJECTORY_STATE_ELLIPSE) {
    t = t - ellipse.t_begin;
    if (ellipse.period > ellipse.goal_period) {
      ellipse.period -= 0.001;
    }
    return ellipse_traj_eval(&ellipse, t, 0.033);
  }
  else {
    return piecewise_eval(ppFront, t, 0.033); //  TODO mass not magic number
  }
}

// works in rest of firmware's structs
void trajectoryGetCurrentGoal(trajectoryPoint_t* goal)
{
  struct traj_eval ev = current_goal();
  pp_eval_to_trajectory_point(&ev, goal);

  if (state == TRAJECTORY_STATE_ELLIPSE) {
    // TODO make it possible for an ellipse to end
  }
  else {
    if (piecewise_is_finished(ppFront)) {
      if (state == TRAJECTORY_STATE_TAKING_OFF) {
        state = TRAJECTORY_STATE_FLYING;
      }
      if (state == TRAJECTORY_STATE_LANDING) {
        state = TRAJECTORY_STATE_IDLE;
      }
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
        // if (!stretched) {
	       // piecewise_stretchtime(ppBack, 0.75);
        //  stretched = true;
        // }
        ret = trajectoryStart();
        break;
      // case COMMAND_STATE:
      //   ret = trajectoryState((const struct data_state*)&p.data[1]);
      case COMMAND_TAKEOFF:
        ret = trajectoryTakeoff((const struct data_takeoff*)&p.data[1]);
        break;
      case COMMAND_LAND:
        ret = trajectoryLand((const struct data_land*)&p.data[1]);
        break;
      case COMMAND_HOVER:
        ret = trajectoryHover((const struct data_hover*)&p.data[1]);
        break;
      case COMMAND_ELLIPSE:
        ret = ellipseStart();
        break;
      default:
        ret = ENOEXEC;
        break;
    }

    //answer
    p.data[3] = ret;
    p.size = 4;
    crtpSendPacket(&p);
  }
}


int trajectoryReset(void)
{
  ppBack->n_pieces = 0;
  // stretched = false;
  // DEBUG_PRINT("trajectoryReset\n");

  return 0;
}

int trajectoryAdd(const struct data_add* data)
{
  if (data->id < PP_MAX_PIECES
      && data->offset + data->size < sizeof(ppBack->pieces[data->id])) {
    uint8_t size = data->size;
    uint8_t* ptr = (uint8_t*)&(data->values[0]);
    uint8_t offset = data->offset;
    if (data->offset == 0) {
      // DEBUG_PRINT("setDur: %d %f\n", data->id, data->values[0]);
      ppBack->pieces[data->id].duration = data->values[0];
      size -= 1;
      ptr += 4;
    } else {
      offset -= 1;
    }
    for (int i = offset; i < offset + size; ++i) {
      ppBack->pieces[data->id].p[i/8][i%8] = data->values[offset == 0 ? i+1 : i-offset];
    }
    // memcpy(ppBack->pieces[data->id].p + offset, ptr, size * sizeof(float));
    ppBack->n_pieces = data->id + 1;
    // DEBUG_PRINT("trajectoryAdd: %d, %d, %d\n", data->id, offset, size);
    return 0;
  }

  return ENOMEM;
}

void trajectoryFlip()
{
  ppBack->t_begin_piece = usecTimestamp() / 1e6;
  ppBack->cursor = 0;

  struct piecewise_traj* tmp = ppFront;
  ppFront = ppBack;
  ppBack = tmp;
  *ppBack = *ppFront;
}


int trajectoryStart(void)
{
  float x, y, z, q0, q1, q2, q3;
  uint16_t last_time_in_ms;
  positionExternalGetLastData(&x, &y, &z, &q0, &q1, &q2, &q3, &last_time_in_ms);

  struct traj_eval traj_init = poly4d_eval(&ppBack->pieces[0], 0, 0.035); // TODO mass param
  struct vec shift_pos = vsub(mkvec(x, y, z), traj_init.pos);
  piecewise_shift_vec(ppBack, shift_pos, 0);

  trajectoryFlip();
  // DEBUG_PRINT("trajectory Start\n");
  // for (int i = 0; i < ppFront->n_pieces; ++i) {
  //   DEBUG_PRINT("Piece: %d (%f s)\n", i, ppFront->pieces[i].duration);
  //   for (int j = 0; j < 4; ++j) {
  //     DEBUG_PRINT("%d: ", j);
  //     for (int k = 0; k < 8; ++k) {
  //       DEBUG_PRINT("%f,", ppFront->pieces[i].p[j][k]);
  //     }
  //     DEBUG_PRINT("\n");
  //   }
  // }

  return 0;
}

int ellipseStart(void)
{
  // TODO
  ellipse.center = mkvec(1, 0, 1);
  ellipse.major = mkvec(-1, 0, 0);
  ellipse.minor = mkvec(0, 0.5, 0);
  ellipse.period = 10;
  ellipse.goal_period = 3;
  ellipse.t_begin = usecTimestamp() / 1e6;
  state = TRAJECTORY_STATE_ELLIPSE;
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

int trajectoryTakeoff(const struct data_takeoff* data)
{
  if (state != TRAJECTORY_STATE_IDLE) {
    return 1;
  }

  float x, y, z, q0, q1, q2, q3;
  uint16_t last_time_in_ms;
  positionExternalGetLastData(&x, &y, &z, &q0, &q1, &q2, &q3, &last_time_in_ms);
  float yaw = quat2rpy(mkquat(q0, q1, q2, q3)).z;

  float delta_z = data->height - z;
  float duration = data->time_from_start / 1000.0;

  struct poly4d p = poly4d_takeoff;
  poly4d_stretchtime(&p, duration / poly4d_takeoff.duration);
  poly4d_scale(&p, 1, 1, delta_z, 1);
  poly4d_shift(&p, x, y, z, 0);
  poly_linear(p.p[3], duration, yaw, 0);

  ppBack->pieces[0] = p;
  ppBack->n_pieces = 1;

  state = TRAJECTORY_STATE_TAKING_OFF;
  // piecewise_stretchtime(ppBack, 2.0);
  trajectoryFlip();
  return 0;
}

int trajectoryLand(const struct data_land* data)
{
  if (state != TRAJECTORY_STATE_FLYING) {
    return 1;
  }

  float x, y, z, q0, q1, q2, q3;
  uint16_t last_time_in_ms;
  positionExternalGetLastData(&x, &y, &z, &q0, &q1, &q2, &q3, &last_time_in_ms);
  float yaw = quat2rpy(mkquat(q0, q1, q2, q3)).z;

  float delta_z = data->height - z;
  float duration = data->time_from_start / 1000.0;

  struct poly4d p = poly4d_takeoff;
  poly4d_stretchtime(&p, duration / poly4d_takeoff.duration);
  poly4d_scale(&p, 1, 1, delta_z, 1);
  poly4d_shift(&p, x, y, z, 0);
  poly_linear(p.p[3], duration, yaw, 0);

  ppBack->pieces[0] = p;
  ppBack->n_pieces = 1;

  state = TRAJECTORY_STATE_LANDING;
  // piecewise_stretchtime(ppBack, 2.0);
  trajectoryFlip();
  return 0;
}

int trajectoryHover(const struct data_hover* data)
{
  if (state != TRAJECTORY_STATE_FLYING && state != TRAJECTORY_STATE_ELLIPSE) {
    return 1;
  }

  struct traj_eval setpoint = current_goal();
  struct vec hover_pos = mkvec(data->x, data->y, data->z);
  float hover_yaw = data->yaw;

  piecewise_plan_5th_order(ppBack, data->duration,
    setpoint.pos, setpoint.yaw, setpoint.vel, setpoint.omega.z, setpoint.acc,
    hover_pos,    hover_yaw,    vzero(),      0,                vzero());

  trajectoryFlip();
  return 0;
}
