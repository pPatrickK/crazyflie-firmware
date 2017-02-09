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
#include "semphr.h"

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
#include "num.h"


// Global variables
static bool isInit = false;
static struct planner planner;
static struct piecewise_traj ppUser;
static uint8_t group;
// makes sure that we don't evaluate the trajectory while it is being changed
// It would be more efficient to put the semaphore in the planning layer, however that would introduce
// FreeRTOS dependencies there...
static xSemaphoreHandle lockTraj;

// Private functions
// TODO consistent naming - should they all start with "trajectory" or not?`
static void trajectoryTask(void * prm);

static int add_poly(const struct data_add_poly* data);
static int start_poly(const struct data_start_poly* data);
static int takeoff(const struct data_takeoff* data);
static int land(const struct data_land* data);
static int hover(const struct data_hover* data);
static int start_ellipse(const struct data_start_ellipse* data);
static int gohome(const struct data_gohome* data);
static int set_ellipse(const struct data_set_ellipse* data);
static int start_canned_trajectory(const struct data_start_canned_trajectory* data);
static int start_avoid_target(const struct data_start_avoid_target* data);
static int set_group(const struct data_set_group* data);

static void posInteractiveCB(const struct vec *pos, const struct quat *quat)
{
  float t = usecTimestamp() / 1e6;
  plan_update_avoid_target(&planner, *pos, t);
}

// hack using global
extern float g_vehicleMass;

static struct vec mkvec_position_fix16_to_float(posFixed16_t x, posFixed16_t y, posFixed16_t z)
{
  return mkvec(
    position_fix16_to_float(x),
    position_fix16_to_float(y),
    position_fix16_to_float(z));
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

bool isGroup(uint8_t g) {
  return g == 0 || g == group;
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

  //initUsecTimer();

  lockTraj = xSemaphoreCreateMutex();

  setPositionInteractiveCallback(&posInteractiveCB);

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

bool trajectoryIsFlying()
{
  return plan_is_flying(&planner);
}

void trajectoryGetCurrentGoal(trajectoryPoint_t* goal)
{
  xSemaphoreTake(lockTraj, portMAX_DELAY);
  float t = usecTimestamp() / 1e6;
  struct traj_eval ev = plan_current_goal(&planner, t);
  if (!is_traj_eval_valid(&ev)) {
    // programming error
    plan_emergency_stop(&planner);
  }
  xSemaphoreGive(lockTraj);
  goal->x = ev.pos.x;
  goal->y = ev.pos.y;
  goal->z = ev.pos.z;
  goal->velocity_x = ev.vel.x;
  goal->velocity_y = ev.vel.y;
  goal->velocity_z = ev.vel.z;
  goal->yaw = ev.yaw;
  goal->omega = ev.omega;
  goal->acceleration = ev.acc;
}

void trajectoryTask(void * prm)
{
  int ret;
  CRTPPacket p;
  crtpInitTaskQueue(CRTP_PORT_TRAJECTORY);

  while(1) {
    crtpReceivePacketBlock(CRTP_PORT_TRAJECTORY, &p);

    switch(p.data[0])
    {
      case COMMAND_RESET_POLY:
        ppUser.n_pieces = 0;
        ret = 0;
        break;
      case COMMAND_ADD_POLY:
        ret = add_poly((const struct data_add_poly*)&p.data[1]);
        break;
      case COMMAND_START_POLY:
        ret = start_poly((const struct data_start_poly*)&p.data[1]);
        break;
      case COMMAND_TAKEOFF:
        ret = takeoff((const struct data_takeoff*)&p.data[1]);
        break;
      case COMMAND_LAND:
        ret = land((const struct data_land*)&p.data[1]);
        break;
      case COMMAND_HOVER:
        ret = hover((const struct data_hover*)&p.data[1]);
        break;
      case COMMAND_START_ELLIPSE:
        ret = start_ellipse((const struct data_start_ellipse*)&p.data[1]);
        break;
      case COMMAND_GOHOME:
        ret = gohome((const struct data_gohome*)&p.data[1]);
        break;
      case COMMAND_SET_ELLIPSE:
        ret = set_ellipse((const struct data_set_ellipse*)&p.data[1]);
        break;
      case COMMAND_START_CANNED_TRAJECTORY:
        ret = start_canned_trajectory((const struct data_start_canned_trajectory*)&p.data[1]);
        break;
      case COMMAND_START_AVOID_TARGET:
        ret = start_avoid_target((const struct data_start_avoid_target*)&p.data[1]);
        break;
      case COMMAND_SET_GROUP:
        ret = set_group((const struct data_set_group*)&p.data[1]);
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

int add_poly(const struct data_add_poly* data)
{
  if (data->id < PP_MAX_PIECES
      && data->offset + data->size < sizeof(ppUser.pieces[data->id])) {
    uint8_t size = data->size;
    uint8_t* ptr = (uint8_t*)&(data->values[0]);
    uint8_t offset = data->offset;
    if (data->offset == 0) {
      // DEBUG_PRINT("setDur: %d %f\n", data->id, data->values[0]);
      ppUser.pieces[data->id].duration = data->values[0];
      size -= 1;
      ptr += 4;
    } else {
      offset -= 1;
    }
    for (int i = offset; i < offset + size; ++i) {
      ppUser.pieces[data->id].p[i/8][i%8] = data->values[offset == 0 ? i+1 : i-offset];
    }
    ppUser.n_pieces = max(ppUser.n_pieces, data->id + 1);
    // DEBUG_PRINT("trajectoryAdd: %d, %d, %d\n", data->id, offset, size);
    return 0;
  }

  return ENOMEM;
}

int start_poly(const struct data_start_poly* data)
{
  if (isGroup(data->group)) {
    xSemaphoreTake(lockTraj, portMAX_DELAY);
    *(planner.ppBack) = ppUser;
    float t = usecTimestamp() / 1e6;
    plan_start_poly(&planner, statePos(), t, data->reversed);
    xSemaphoreGive(lockTraj);
  }
  return 0;
}

int takeoff(const struct data_takeoff* data)
{
  int result = 0;
  if (isGroup(data->group)) {
    float duration = data->time_from_start / 1000.0;
    xSemaphoreTake(lockTraj, portMAX_DELAY);
    float t = usecTimestamp() / 1e6;
    result = plan_takeoff(&planner, statePos(), stateYaw(), data->height, duration, t);
    xSemaphoreGive(lockTraj);
  }
  return result;
}

int land(const struct data_land* data)
{
  int result = 0;
  if (isGroup(data->group)) {
    float duration = data->time_from_start / 1000.0;
    xSemaphoreTake(lockTraj, portMAX_DELAY);
    float t = usecTimestamp() / 1e6;
    result = plan_land(&planner, statePos(), stateYaw(), data->height, duration, t);
    xSemaphoreGive(lockTraj);
  }
  return result;
}

int hover(const struct data_hover* data)
{
  struct vec hover_pos = mkvec(data->x, data->y, data->z);
  xSemaphoreTake(lockTraj, portMAX_DELAY);
  float t = usecTimestamp() / 1e6;
  int result = plan_hover(&planner, hover_pos, data->yaw, data->duration, t);
  xSemaphoreGive(lockTraj);
  return result;
}

int start_ellipse(const struct data_start_ellipse* data)
{
  if (isGroup(data->group)) {
    xSemaphoreTake(lockTraj, portMAX_DELAY);
    float t = usecTimestamp() / 1e6;
    plan_start_ellipse(&planner, t);
    xSemaphoreGive(lockTraj);
  }
  return 0;
}

int gohome(const struct data_gohome* data)
{
  if (isGroup(data->group)) {
    xSemaphoreTake(lockTraj, portMAX_DELAY);
    float t = usecTimestamp() / 1e6;
    plan_go_home(&planner, t);
    xSemaphoreGive(lockTraj);
  }
  return 0;
}

int set_ellipse(const struct data_set_ellipse* data)
{
  planner.ellipse.center = mkvec_position_fix16_to_float(
    data->centerx, data->centery, data->centerz);
  planner.ellipse.major = mkvec_position_fix16_to_float(
    data->majorx, data->majory, data->majorz);
  planner.ellipse.minor = mkvec_position_fix16_to_float(
    data->minorx, data->minory, data->minorz);
  planner.ellipse.period = data->period;

  return 0;
}

int start_canned_trajectory(const struct data_start_canned_trajectory* data)
{
  int result = 0;
  if (isGroup(data->group)) {
    xSemaphoreTake(lockTraj, portMAX_DELAY);
    float t = usecTimestamp() / 1e6;
    result = plan_start_canned_trajectory(&planner,
      data->trajectory, data->timescale, statePos(), t);
    xSemaphoreGive(lockTraj);
  }
  return result;
}

int start_avoid_target(const struct data_start_avoid_target* data)
{
  struct vec home = mkvec(data->x, data->y, data->z);
  xSemaphoreTake(lockTraj, portMAX_DELAY);
  float t = usecTimestamp() / 1e6;
  plan_start_avoid_target(&planner, home, data->max_displacement, data->max_speed, t);
  xSemaphoreGive(lockTraj);
  return 0;
}

int set_group(const struct data_set_group* data)
{
  group = data->group;

  return 0;
}
