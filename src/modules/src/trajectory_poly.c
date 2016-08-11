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
#include "crtp.h"
#include "trajectory.h"
#include "debug.h"
#include "mathconstants.h"
#include "position_external.h"
#include "packetdef.h"
#include "planner.h"
#include "usec_time.h"
#include "log.h"
#include "stabilizer.h" // to get current state estimate


// Global variables
static bool isInit = false;
static CRTPPacket p;
static struct planner planner;

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

  plan_init(&planner, g_vehicleMass);

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

void trajectoryStop()
{
  plan_emergency_stop(&planner);
}

bool trajectoryIsStopped()
{
  return plan_is_stopped(&planner);
}

void trajectoryGetCurrentGoal(trajectoryPoint_t* goal)
{
  float t = usecTimestamp() / 1e6;
  struct traj_eval ev = plan_current_goal(&planner, t);
  if (!is_traj_eval_valid(&ev)) {
    // programming error
    plan_emergency_stop(&planner);
  }
  goal->x = ev.pos.x;
  goal->y = ev.pos.y;
  goal->z = ev.pos.z;
  goal->velocity_x = ev.vel.x;
  goal->velocity_y = ev.vel.y;
  goal->velocity_z = ev.vel.z;
  goal->yaw = ev.yaw;
  goal->omega = ev.omega;
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
  planner.ppBack->n_pieces = 0;
  return 0;
}

int trajectoryAddPoly(const struct data_add_poly* data)
{
  if (data->id < PP_MAX_PIECES
      && data->offset + data->size < sizeof(planner.ppBack->pieces[data->id])) {
    uint8_t size = data->size;
    uint8_t* ptr = (uint8_t*)&(data->values[0]);
    uint8_t offset = data->offset;
    if (data->offset == 0) {
      // DEBUG_PRINT("setDur: %d %f\n", data->id, data->values[0]);
      planner.ppBack->pieces[data->id].duration = data->values[0];
      size -= 1;
      ptr += 4;
    } else {
      offset -= 1;
    }
    for (int i = offset; i < offset + size; ++i) {
      planner.ppBack->pieces[data->id].p[i/8][i%8] = data->values[offset == 0 ? i+1 : i-offset];
    }
    planner.ppBack->n_pieces = data->id + 1;
    // DEBUG_PRINT("trajectoryAdd: %d, %d, %d\n", data->id, offset, size);
    return 0;
  }

  return ENOMEM;
}

int trajectoryStartPoly(void)
{
  float t = usecTimestamp() / 1e6;
  plan_start_poly(&planner, statePos(), t);
  return 0;
}

int startEllipse(void)
{
  float t = usecTimestamp() / 1e6;
  plan_start_ellipse(&planner, t);
  return 0;
}

int trajectoryTakeoff(const struct data_takeoff* data)
{
  float t = usecTimestamp() / 1e6;
  float duration =  data->time_from_start / 1000.0;
  return plan_takeoff(&planner, statePos(), stateYaw(), data->height, duration, t);
}

int trajectoryLand(const struct data_land* data)
{
  float t = usecTimestamp() / 1e6;
  float duration =  data->time_from_start / 1000.0;
  return plan_land(&planner, statePos(), stateYaw(), data->height, duration, t);
}

int trajectoryHover(const struct data_hover* data)
{
  float t = usecTimestamp() / 1e6;
  struct vec hover_pos = mkvec(data->x, data->y, data->z);
  return plan_hover(&planner, hover_pos, data->yaw, data->duration, t);
}

int goHome(void)
{
  float t = usecTimestamp() / 1e6;
  return plan_go_home(&planner, t);
}

int setEllipse(const struct data_set_ellipse* data)
{
  planner.ellipse.center = mkvec_position_fix2float(
    data->centerx, data->centery, data->centerz);
  planner.ellipse.major = mkvec_position_fix2float(
    data->majorx, data->majory, data->majorz);
  planner.ellipse.minor = mkvec_position_fix2float(
    data->minorx, data->minory, data->minorz);
  planner.ellipse.period = data->period;

  return 0;
}

int startCannedTrajectory(const struct data_start_canned_trajectory* data)
{
  enum trajectory_type type = data->trajectory;
  switch (type) {
  case TRAJECTORY_FIGURE8:
    *planner.ppBack = pp_figure8;
    break;
  default:
    return 1;
  }

  float t = usecTimestamp() / 1e6;
  plan_start_poly(&planner, statePos(), t);
  return 0;
}

int startAvoidTarget(const struct data_start_avoid_target* data)
{
  float t = usecTimestamp() / 1e6;
  struct vec home = mkvec(data->x, data->y, data->z);
  plan_start_avoid_target(&planner, home, data->max_displacement, data->max_speed, t);
  return 0;
}

void updateAvoidTarget(const struct position_message *msg)
{
  float t = usecTimestamp() / 1e6;
  plan_update_avoid_target(&planner, msg->pos, t);
}
