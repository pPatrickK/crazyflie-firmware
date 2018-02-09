/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
 *
 */
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "commander.h"
#include "crtp_commander.h"
#include "trajectory.h"
#include "param.h"

static bool isInit;
const static setpoint_t nullSetpoint;
const static int priorityDisable = COMMANDER_PRIORITY_DISABLE;

static uint32_t lastUpdate;

static bool enableTrajectory = false;

QueueHandle_t setpointQueue;
QueueHandle_t priorityQueue;

/* Public functions */
void commanderInit(void)
{
  setpointQueue = xQueueCreate(1, sizeof(setpoint_t));
  ASSERT(setpointQueue);
  xQueueSend(setpointQueue, &nullSetpoint, 0);

  priorityQueue = xQueueCreate(1, sizeof(int));
  ASSERT(priorityQueue);
  xQueueSend(priorityQueue, &priorityDisable, 0);

  crtpCommanderInit();
  trajectoryInit();

  lastUpdate = xTaskGetTickCount();

  isInit = true;
}

void commanderSetSetpoint(setpoint_t *setpoint, int priority)
{
  int currentPriority;
  xQueuePeek(priorityQueue, &currentPriority, 0);

  if (priority >= currentPriority) {
    setpoint->timestamp = xTaskGetTickCount();
    // This is a potential race but without effect on functionality
    xQueueOverwrite(setpointQueue, setpoint);
    xQueueOverwrite(priorityQueue, &priority);
  }
}

void commanderGetSetpoint(setpoint_t *setpoint, const state_t *state)
{
  if (enableTrajectory) {
    trajectorySetState(state);
    if (trajectoryIsStopped()) {
      memcpy(setpoint, &nullSetpoint, sizeof(nullSetpoint));
    } else {
      // update setpoint
      trajectoryPoint_t goal;
      trajectoryGetCurrentGoal(&goal);

      setpoint->position.x = goal.x;
      setpoint->position.y = goal.y;
      setpoint->position.z = goal.z;
      setpoint->velocity.x = goal.velocity_x;
      setpoint->velocity.y = goal.velocity_y;
      setpoint->velocity.z = goal.velocity_z;
      setpoint->attitude.yaw = degrees(goal.yaw);
      setpoint->attitudeRate.roll = degrees(goal.omega.x);
      setpoint->attitudeRate.pitch = degrees(goal.omega.y);
      setpoint->attitudeRate.yaw = degrees(goal.omega.z);
      setpoint->mode.x = modeAbs;
      setpoint->mode.y = modeAbs;
      setpoint->mode.z = modeAbs;
      setpoint->mode.roll = modeDisable;
      setpoint->mode.pitch = modeDisable;
      setpoint->mode.yaw = modeAbs;
      setpoint->mode.quat = modeDisable;
      setpoint->acceleration.x = goal.acceleration.x;
      setpoint->acceleration.y = goal.acceleration.y;
      setpoint->acceleration.z = goal.acceleration.z;
    }

    return;
  }

  xQueuePeek(setpointQueue, setpoint, 0);
  lastUpdate = setpoint->timestamp;
  uint32_t currentTime = xTaskGetTickCount();

  if ((currentTime - setpoint->timestamp) > COMMANDER_WDT_TIMEOUT_SHUTDOWN) {
    memcpy(setpoint, &nullSetpoint, sizeof(nullSetpoint));
  } else if ((currentTime - setpoint->timestamp) > COMMANDER_WDT_TIMEOUT_STABILIZE) {
    xQueueOverwrite(priorityQueue, &priorityDisable);
    // Leveling ...
    setpoint->mode.x = modeDisable;
    setpoint->mode.y = modeDisable;
    setpoint->mode.roll = modeAbs;
    setpoint->mode.pitch = modeAbs;
    setpoint->mode.yaw = modeVelocity;
    setpoint->attitude.roll = 0;
    setpoint->attitude.pitch = 0;
    setpoint->attitudeRate.yaw = 0;
    // Keep Z as it is
  }
}

bool commanderTest(void)
{
  return isInit;
}

uint32_t commanderGetInactivityTime(void)
{
  return xTaskGetTickCount() - lastUpdate;
}

int commanderGetActivePriority(void)
{
  int priority;
  xQueuePeek(priorityQueue, &priority, 0);
  return priority;
}

PARAM_GROUP_START(flightmode)
PARAM_ADD(PARAM_UINT8, posCtrl, &enableTrajectory)
PARAM_GROUP_STOP(flightmode)