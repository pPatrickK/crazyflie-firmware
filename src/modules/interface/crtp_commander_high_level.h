#ifndef CRTP_COMMANDER_HIGH_LEVEL_H_
#define CRTP_COMMANDER_HIGH_LEVEL_H_

#include <stdbool.h>
#include <stdint.h>

#include "math3d.h"

#include "stabilizer_types.h"

/* Public functions */
void crtpCommanderHighLevelInit(void);

// Retrieves the current setpoint
void crtpCommanderHighLevelGetSetpoint(setpoint_t* setpoint, const state_t *state);

// Tell the trajectory planner that it should cut power.
// Should be used if an emergency is detected.
void crtpCommanderHighLevelStop();

// True if we have landed or emergency-stopped.
bool crtpCommanderHighLevelIsStopped();

// True if we are flying (i.e. not stopped and not during landing or takeoff)
bool crtpCommanderHighLevelIsFlying();


#endif /* CRTP_COMMANDER_HIGH_LEVEL_H_ */
