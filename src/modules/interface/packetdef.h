/**
 *    ______
 *   / ____/________ _____  __  ________      ______ __________ ___
 *  / /   / ___/ __ `/_  / / / / / ___/ | /| / / __ `/ ___/ __ `__ \
 * / /___/ /  / /_/ / / /_/ /_/ (__  )| |/ |/ / /_/ / /  / / / / / /
 * \____/_/   \__,_/ /___/\__, /____/ |__/|__/\__,_/_/  /_/ /_/ /_/
 *                       /____/
 *
 * Crazyswarm advanced control firmware for Crazyflie
 *
 * Copyright (C) 2016 Wolfgang Hoenig and James Preiss,
 * University of Southern California
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
 * packetdef.h : definitions of all CRTP packets used in Crazyswarm.
                 centralized so it can be shared between server and vehicle.
 */


#include <stdint.h>



// ----------------------------------------------------------------------- //
//                            Vicon Positions                              //
//                       arrive on CRTP_PORT_POSEXT                        //
// ----------------------------------------------------------------------- //

static float const POSITION_LIMIT = 8.0f; // meters

static inline int16_t position_float2fix(float x)
{
  return (INT16_MAX / POSITION_LIMIT) * x;
}

static inline float position_fix2float(int16_t x)
{
  return (POSITION_LIMIT / INT16_MAX) * ((float)x);
}

typedef uint16_t fp16_t;
struct data_vicon {
  struct {
    uint8_t id;
    int16_t x; // m
    int16_t y; // m
    int16_t z; // m
    uint32_t quat; // compressed quat, see quatcompress.h
  } pose[1];
} __attribute__((packed));


// ----------------------------------------------------------------------- //
//                              Trajectories                               //
//                     arrive on CRTP_PORT_TRAJECTORY                      //
// ----------------------------------------------------------------------- //

// TODO explain where this is used
enum TrajectoryCommand_e {
  COMMAND_RESET   = 0,
  COMMAND_ADD     = 1,
  COMMAND_START   = 2,
  //COMMAND_STATE = 3,
  COMMAND_TAKEOFF = 4,
  COMMAND_LAND    = 5,
  COMMAND_HOVER   = 6,
  COMMAND_ELLIPSE = 7,
};

// multi-packet piecewise polynomial definition
struct data_add {
  uint8_t id;
  uint8_t offset:5;
  uint8_t size:3;
  float values[6];
} __attribute__((packed));


// "take this much time to go here, then hover"
struct data_hover {
  float x; // m
  float y; // m
  float z; // m
  float yaw; // deg
  float duration; // sec
} __attribute__((packed));


// vertical takeoff from current x-y position to given height
struct data_takeoff {
  float height; // m (absolute)
  uint16_t time_from_start; // ms
} __attribute__((packed));


// vertical land from current x-y position to given height
struct data_land {
  float height; // m (absolute)
  uint16_t time_from_start; // ms
} __attribute__((packed));

