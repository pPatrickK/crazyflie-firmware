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

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"

// Crazyswarm includes
#include "avoidtarget.h"
#include "crtp.h"
#include "trajectory.h"
#include "debug.h"
#include "mathconstants.h"
#include "position_external.h"
#include "pptraj.h"
#include "packetdef.h"
#include "usec_time.h"
#include "log.h"
#include "stabilizer.h" // to get current state estimate


// Global variables
static bool isInit = false;
static CRTPPacket p;
static trajectoryState_t state = TRAJECTORY_STATE_IDLE;

static struct piecewise_traj* ppFront;
static struct piecewise_traj* ppBack;
static struct piecewise_traj pp1;
static struct piecewise_traj pp2;

static struct ellipse_traj ellipse;
static struct avoid_target avoid;

struct vec home;


// Private functions
// TODO consistent naming - should they all start with "trajectory" or not?`
static void trajectoryTask(void * prm);
static int trajectoryResetPoly(void);
static int trajectoryAddPoly(const struct data_add_poly* data);
static int trajectoryStartPoly(void);
// static int trajectoryState(const struct data_state* data);
static int trajectoryTakeoff();
static int trajectoryLand();
static int trajectoryHover(const struct data_hover* data);
static int startEllipse();
static int goHome();
static int setEllipse(const struct data_set_ellipse* data);
static int startCannedTrajectory(const struct data_start_canned_trajectory* data);
static int startAvoidTarget(const struct data_start_avoid_target* data);
static void updateAvoidTarget(const struct position_message *msg);

// hack using global
extern float g_vehicleMass;

static struct vec mkvec_position_fix2float(posFixed16_t x, posFixed16_t y, posFixed16_t z)
{
  return mkvec(
    position_fix2float(x),
    position_fix2float(y),
    position_fix2float(z));
}

static struct vec state2vec(struct vec3_s v)
{
  return mkvec(v.x, v.y, v.z);
}

static struct vec statePos()
{
	return state2vec(stabilizerState()->position);
}

static float stateYaw()
{
	return radians(stabilizerState()->attitude.yaw);
}

void trajectoryInit(void)
{
  if (isInit) {
    return;
  }

  ppFront = &pp1;
  ppBack = &pp2;

  //Start the trajectory task
  xTaskCreate(trajectoryTask, TRAJECTORY_TASK_NAME,
              TRAJECTORY_TASK_STACKSIZE, NULL, TRAJECTORY_TASK_PRI, NULL);

  positionInteractiveSetCallback(&updateAvoidTarget);

  initUsecTimer();
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
  switch (state) {
  // NOTE using return instead of break
  case TRAJECTORY_STATE_ELLIPSE:
    t = t - ellipse.t_begin;
    return ellipse_traj_eval(&ellipse, t, g_vehicleMass);
  case TRAJECTORY_STATE_AVOID_TARGET:
    return eval_avoid_target(&avoid, t);
  default:
    return piecewise_eval(ppFront, t, g_vehicleMass);
  }
}

// works in rest of firmware's structs, updates states
void trajectoryGetCurrentGoal(trajectoryPoint_t* goal)
{
  float t = usecTimestamp() / 1e6;
  struct traj_eval ev = current_goal();
  pp_eval_to_trajectory_point(&ev, goal);

  if (state == TRAJECTORY_STATE_ELLIPSE_CATCHUP) {
    if (piecewise_is_finished(ppFront, t)) {
      state = TRAJECTORY_STATE_ELLIPSE;
    }
  }
  else if (state == TRAJECTORY_STATE_ELLIPSE) {
    // TODO make it possible for an ellipse to end
  }
  else if (state == TRAJECTORY_STATE_AVOID_TARGET) {
    // TODO anything??
  }
  else {
    // FLYING, TAKING_OFF, or LANDING
    if (piecewise_is_finished(ppFront, t)) {
      if (state == TRAJECTORY_STATE_TAKING_OFF) {
        state = TRAJECTORY_STATE_FLYING;
      }
      if (state == TRAJECTORY_STATE_LANDING) {
        state = TRAJECTORY_STATE_IDLE;
      }
    }
  }
}

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
      case COMMAND_RESET_POLY:
        ret = trajectoryResetPoly();
        break;
      case COMMAND_ADD_POLY:
        ret = trajectoryAddPoly((const struct data_add_poly*)&p.data[1]);
        break;
      case COMMAND_START_POLY:
        ret = trajectoryStartPoly();
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
      case COMMAND_START_ELLIPSE:
        ret = startEllipse();
        break;
      case COMMAND_GOHOME:
        ret = goHome();
        break;
      case COMMAND_SET_ELLIPSE:
        ret = setEllipse((const struct data_set_ellipse*)&p.data[1]);
        break;
      case COMMAND_START_CANNED_TRAJECTORY:
        ret = startCannedTrajectory((const struct data_start_canned_trajectory*)&p.data[1]);
        break;
      case COMMAND_START_AVOID_TARGET:
        ret = startAvoidTarget((const struct data_start_avoid_target*)&p.data[1]);
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


int trajectoryResetPoly(void)
{
  ppBack->n_pieces = 0;
  return 0;
}

int trajectoryAddPoly(const struct data_add_poly* data)
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
    ppBack->n_pieces = data->id + 1;
    // DEBUG_PRINT("trajectoryAdd: %d, %d, %d\n", data->id, offset, size);
    return 0;
  }

  return ENOMEM;
}

void trajectoryFlip()
{
  ppBack->t_begin = usecTimestamp() / 1e6;
  struct piecewise_traj* tmp = ppFront;
  ppFront = ppBack;
  ppBack = tmp;
  // this is necessary because we might recieve the startTrajectory command
  // repeatedly in a short time... TODO avoid doing this
  *ppBack = *ppFront;
}

int trajectoryStartPoly(void)
{
  struct traj_eval traj_init = poly4d_eval(&ppBack->pieces[0], 0, g_vehicleMass);
  struct vec shift_pos = vsub(statePos(), traj_init.pos);
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

int startEllipse(void)
{
  ellipse.t_begin = usecTimestamp() / 1e6;
  struct traj_eval ev_current = current_goal();
  plan_into_ellipse(&ev_current, &ellipse, ppBack, g_vehicleMass);
  trajectoryFlip();
  state = TRAJECTORY_STATE_ELLIPSE_CATCHUP;
  return 0;
}

static void planTakeoffLanding(float height, float duration)
{
  struct vec pos = statePos();
  float yaw = stateYaw();
  float delta_z = height - pos.z;

  struct poly4d p = poly4d_takeoff;
  poly4d_stretchtime(&p, duration / poly4d_takeoff.duration);
  poly4d_scale(&p, 1, 1, delta_z, 1);
  poly4d_shift_vec(&p, pos, 0);
  poly_linear(p.p[3], duration, yaw, 0);

  ppBack->pieces[0] = p;
  ppBack->n_pieces = 1;
}

int trajectoryTakeoff(const struct data_takeoff* data)
{
  if (state != TRAJECTORY_STATE_IDLE) {
    return 1;
  }

  home = statePos();
  planTakeoffLanding(data->height, data->time_from_start / 1000.0);
  state = TRAJECTORY_STATE_TAKING_OFF;
  trajectoryFlip();
  return 0;
}

int trajectoryLand(const struct data_land* data)
{
  // TODO this means we can't land from ELLIPSE or AVOID_TARGET
  // states... you must GO_HOME first. Is this good?
  if (state != TRAJECTORY_STATE_FLYING) {
    return 1;
  }

  planTakeoffLanding(data->height, data->time_from_start / 1000.0);
  state = TRAJECTORY_STATE_LANDING;
  trajectoryFlip();
  return 0;
}

int trajectoryHover(const struct data_hover* data)
{
  if (state != TRAJECTORY_STATE_FLYING &&
      state != TRAJECTORY_STATE_ELLIPSE &&
      state != TRAJECTORY_STATE_AVOID_TARGET) {
    return 1;
  }

  struct traj_eval setpoint = current_goal();
  struct vec hover_pos = mkvec(data->x, data->y, data->z);
  float hover_yaw = data->yaw;

  // TODO try out 7th order planner
  piecewise_plan_5th_order(ppBack, data->duration,
    setpoint.pos, setpoint.yaw, setpoint.vel, setpoint.omega.z, setpoint.acc,
    hover_pos,    hover_yaw,    vzero(),      0,                vzero());

  state = TRAJECTORY_STATE_FLYING;
  trajectoryFlip();
  return 0;
}

int goHome(void)
{
  struct data_hover data;
  data.x = home.x;
  data.y = home.y;
  data.z = home.z;
  data.yaw = 0;
  data.duration = 2;

  return trajectoryHover(&data);
}

int setEllipse(const struct data_set_ellipse* data)
{
  ellipse.center = mkvec_position_fix2float(
    data->centerx, data->centery, data->centerz);
  ellipse.major = mkvec_position_fix2float(
    data->majorx, data->majory, data->majorz);
  ellipse.minor = mkvec_position_fix2float(
    data->minorx, data->minory, data->minorz);
  ellipse.period = data->period;

  return 0;
}

int startCannedTrajectory(const struct data_start_canned_trajectory* data)
{
  enum trajectory_type type = data->trajectory;
  switch (type) {
  case TRAJECTORY_FIGURE8:
    *ppBack = pp_figure8;
    break;
  default:
    return 1;
  }

  struct traj_eval traj_init = poly4d_eval(&ppBack->pieces[0], 0, g_vehicleMass);
  struct vec shift_pos = vsub(statePos(), traj_init.pos);
  piecewise_shift_vec(ppBack, shift_pos, 0);
  piecewise_stretchtime(ppBack, data->timescale);

  trajectoryFlip();

  return 0;
}

int startAvoidTarget(const struct data_start_avoid_target* data)
{
  struct vec home = mkvec(data->x, data->y, data->z);
  float t = usecTimestamp() / 1e6;
  init_avoid_target(&avoid, home, data->max_speed, data->max_displacement, t);
  state = TRAJECTORY_STATE_AVOID_TARGET;
  return 0;
}

void updateAvoidTarget(const struct position_message *msg)
{
  float t = usecTimestamp() / 1e6;
  update_avoid_target(&avoid, msg->pos, t);
}
