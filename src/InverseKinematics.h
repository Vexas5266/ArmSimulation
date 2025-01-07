#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include <math.h>
#include "RoveMatrix.h"

#define RAD2DEG (180.0f / M_PI)
#define DEG2RAD (M_PI / 180.0f)

#define J2_LENGTH 18
#define J3_LENGTH 18.5

#define J1_FWD_LIM 6.3
#define J1_REV_LIM -6.3

#define J2_FWD_LIM 164
#define J2_REV_LIM -54

#define J3_POS_LIM 90
#define J3_NEG_LIM -116.8
#define J3_MID_LIM 15

#define PITCH_FWD_LIM 350
#define PITCH_REV_LIM 10

//If returns false, calculated angle targets are outside of range. HoldCurrentPosition() can then be called
bool CalculateInverseKinematics(Vector pos, float wristPitch, float wristValkyrie, float &q1, float &q2, float &q3, float &q4, float &qP, float &qV, float &J3FwdLim, float &J3RevLim);
bool isOutsideTargetRange(float fwdLim, float revLim, float angle);

#endif /*INVERSE_KINEMATICS_H*/