
#include "stabilizer.h"
#include "stabilizer_types.h"

#include "sensfusion6.h"
#include "position_estimator.h"
#include "mathconstants.h"

#define ATTITUDE_UPDATE_RATE RATE_250_HZ
#define ATTITUDE_UPDATE_DT (1.0/ATTITUDE_UPDATE_RATE)

#define POS_UPDATE_RATE RATE_100_HZ
#define POS_UPDATE_DT (1.0/POS_UPDATE_RATE)

static struct vec3_s oldPosition;
static bool firstUpdate = true;
static float oldYaw = 0.0;

void stateEstimatorInit(void)
{
  sensfusion6Init();
}

bool stateEstimatorTest(void)
{
  bool pass = true;

  pass &= sensfusion6Test();

  return pass;
}

void stateEstimator(state_t *state, const sensorData_t *sensorData, const uint32_t tick)
{
  if (RATE_DO_EXECUTE(ATTITUDE_UPDATE_RATE, tick)) {
    sensfusion6UpdateQ(sensorData->gyro.x, sensorData->gyro.y, sensorData->gyro.z,
                       sensorData->acc.x, sensorData->acc.y, sensorData->acc.z,
                       ATTITUDE_UPDATE_DT);
    // float yaw;
    sensfusion6GetEulerRPY(&state->attitude.roll, &state->attitude.pitch, &state->attitude.yaw);

    sensfusion6GetQuaternion(&state->attitude_q.x, &state->attitude_q.y, &state->attitude_q.z, &state->attitude_q.w);

    // state->attitude.yaw += (yaw - oldYaw);
    // oldYaw = yaw;

    state->acc.z = sensfusion6GetAccZWithoutGravity(sensorData->acc.x,
                                                    sensorData->acc.y,
                                                    sensorData->acc.z);

    positionUpdateVelocity(state->acc.z, ATTITUDE_UPDATE_DT);

    state->attitudeRate.roll = sensorData->gyro.x / 180.0 * M_PI;
    state->attitudeRate.pitch = -sensorData->gyro.y / 180.0 * M_PI;
    state->attitudeRate.yaw = sensorData->gyro.z / 180.0 * M_PI;
  }

  if (RATE_DO_EXECUTE(POS_UPDATE_RATE, tick)) {
    // If position sensor data is preset, pass it throught
    // FIXME: The position sensor shall be used as an input of the estimator
    if (sensorData->position.timestamp) {
      state->position = sensorData->position;
    } else {
      positionEstimate(state, sensorData->baro.asl, POS_UPDATE_DT);
    }

    // Fuse VICON yaw with gyro
    // const float alpha = 0.99;
    // float yaw1 = state->attitude.yaw;
    // float yaw2 = sensorData->external_yaw * 180.0 / M_PI;
    // state->attitude.yaw = alpha * yaw1  + (1-alpha) * yaw2;

    // Update velocity
    if (!firstUpdate) {
      state->velocity.x = (sensorData->position.x - oldPosition.x) / POS_UPDATE_DT;
      state->velocity.y = (sensorData->position.y - oldPosition.y) / POS_UPDATE_DT;
      state->velocity.z = (sensorData->position.z - oldPosition.z) / POS_UPDATE_DT;
    }

    // update state
    oldPosition = sensorData->position;
    firstUpdate = false;
  }
}
