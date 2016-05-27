#pragma once

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

#ifndef M_PI_2
#define M_PI_2 (3.14159265358979323846264338327950288/2)
#endif

#define GRAV 9.81

// measured constants
#define VICON_VAR_XY 1.5e-7
// #define VICON_VAR_Z  1.0e-8
#define VICON_VAR_Q  4.5e-4
#define GYRO_VAR_XYZ 0.2e-6
// #define ACC_VAR_XY   1.5e-5
// #define ACC_VAR_Z    3.9e-5
// the accelerometer variance in z was quite a bit higher
// but to keep the code simple for now we just average them
#define ACC_VAR_XYZ  2.4e-3
