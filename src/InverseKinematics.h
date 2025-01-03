#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include <math.h>
#include "RoveMatrix.h"

#define RAD_TO_DEG (180.0f / M_PI)
#define DEG_TO_RAD (M_PI / 180.0f)

void CalculateInverseKinematics(Vector pos, float &q1, float &q2, float &q3, float &q4, float &qP, float &qV);

#endif /*INVERSE_KINEMATICS_H*/