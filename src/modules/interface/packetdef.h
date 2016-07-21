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
// ----------------------------------------------------------------------- //

typedef uint16_t fp16_t;
struct data_vicon {
  struct {
    uint8_t id;
    fp16_t x; // m
    fp16_t y; // m
    fp16_t z; // m
    int16_t quat[4]; //Quaternion; TODO: find more compact way to store this
                      // each component between -1 and 1
  } pose[1];
};


// ----------------------------------------------------------------------- //
//                              Trajectories                               //
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

