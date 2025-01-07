#include "InverseKinematics.h"

bool CalculateInverseKinematics(Vector pos, float wristPitch, float wristValkyrie, float &q1, float &q2, float &q3, float &q4, float &qP, float &qV, float &J3FwdLim, float &J3RevLim) 
{
    
	//Calculate which solution to use
	bool underMode = (q3 > 0)? true : false;

	//Limit J3 based on which solution is used
	if (underMode) {
		J3FwdLim = J3_POS_LIM;
		J3RevLim = J3_MID_LIM;
	} else {
		J3FwdLim = (-1)*J3_MID_LIM;
		J3RevLim = J3_NEG_LIM;
	}

	//Calculate target angles using IK
	q1 = pos.z;
	q3 = RAD2DEG*acos((pow(pos.x,2)+pow(pos.y,2)-pow(J2_LENGTH,2)-pow(J3_LENGTH,2))/(2*J2_LENGTH*J3_LENGTH));

	if (underMode) q2 = RAD2DEG*(atan2(pos.y, pos.x) - atan2(J3_LENGTH*sin(q3*DEG2RAD),J2_LENGTH+(J3_LENGTH*cos(q3*DEG2RAD))));
	else q2 = RAD2DEG*(atan2(pos.y, pos.x) + atan2(J3_LENGTH*sin(q3*DEG2RAD),J2_LENGTH+(J3_LENGTH*cos(q3*DEG2RAD))));
	
	q3 = underMode? q3 : -q3;

	//Calculate spherical wrist
	q4 = 90; //Lock J4
	qP = wristPitch - (q2 + q3);
	qV = wristValkyrie;

	// Check if calculated angle is invalid and limit movement
	if (isOutsideTargetRange(J1_FWD_LIM, J1_REV_LIM, q1) || isOutsideTargetRange(J2_FWD_LIM, J2_REV_LIM, q2) || isOutsideTargetRange(J3FwdLim, J3RevLim, q3) || isOutsideTargetRange(PITCH_FWD_LIM, PITCH_REV_LIM, qP)) return false;
    return true;
}

bool isOutsideTargetRange(float fwdLim, float revLim, float angle)
{
    if (angle > fwdLim) return true;
	if (angle < revLim) return true;
	return false;
}