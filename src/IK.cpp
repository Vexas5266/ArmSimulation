#include "IK.h"

void IK::Draw() {
	DrawModel(J1Model, (Vector3){0.0f, 0.0f, 0.0f }, 1.0f, RED);		
	DrawModel(J2Model, (Vector3){0, 0, 0}, 1.0f, BLUE);
	DrawModel(J4Model, (Vector3){0, 0, 0}, 1.0f, ORANGE);
	DrawModel(PitchModel, (Vector3){0, 0, 0}, 1.0f, PURPLE);
	DrawModel(ValkModel, (Vector3){0, 0, 0}, 1.0f, GREEN);
	DrawModel(SolModel, (Vector3){0, 0, 0}, 1.0f, MAROON);
	DrawSphere(GripperPos, 2, MAROON);
	DrawGrid(100, 10.0f);

	EndMode3D();

	DrawText("Arm Simulation", 5,5,20,BLACK);
	DrawText("CM: ", 5,700,20,BLACK);
	if (controlMode == OPEN_LOOP) DrawText("O", 150,700,20,BLACK);
	else if (controlMode == CLOSED_LOOP) DrawText("C", 150,700,20,BLACK);
	else DrawText("IK", 150,700,20,BLACK);
	DrawText("X: ", 5,650,20,BLACK);
	DrawText(TextFormat("%.2f", WristPos.x), 40,650,20,BLACK);
	DrawText("Y: ", 150,650,20,BLACK);
	DrawText(TextFormat("%.2f", WristPos.y), 190,650,20,BLACK);
	DrawText("B: ", 5,600,20,BLACK);
	DrawText(TextFormat("%d", buttonInput), 60,600,20,BLACK);
	// DrawText("qVt: ", 150,600,20,BLACK);
	// DrawText(TextFormat("%.2f", RAD2DEG*qV.target), 210,600,20,BLACK);

	EndDrawing();
}

void IK::Unload() {
	UnloadModel(J1Model);
	UnloadModel(J2Model);
	UnloadModel(J4Model);
	UnloadModel(PitchModel);
	UnloadModel(ValkModel);
	UnloadModel(SolModel);
}

void IK::Transform() {
	J1Model.transform = MatrixRotateZ(-q2.motor) * MatrixTranslate(0, 0, INTOPIXELS*q1.motor);
	J2Model.transform = MatrixRotateZ(-q3.motor) * MatrixTranslate(-J2_LENGTH*INTOPIXELS, 0, 0) * J1Model.transform;
	J4Model.transform = MatrixRotateX(-q4.motor) * MatrixTranslate(-SHOULDER_LENGTH*INTOPIXELS, 0, 0) * J2Model.transform;
	PitchModel.transform = MatrixRotateZ(-qP.motor) * MatrixTranslate(-(J3_LENGTH-SHOULDER_LENGTH)*INTOPIXELS, 0, 0) * J4Model.transform;
	ValkModel.transform = MatrixRotateX(-qV.motor) * MatrixTranslate(-WRIST_RAD*INTOPIXELS, 0, 0) * PitchModel.transform;
	SolModel.transform = MatrixRotateY(M_PI) * MatrixTranslate(WRIST_RAD*INTOPIXELS, 0, 0) * PitchModel.transform;
}

void IK::Keyboard() {
	if (IsKeyPressed(KEY_M)) {
		if (controlMode == OPEN_LOOP) controlMode = CLOSED_LOOP;
		else if (controlMode == CLOSED_LOOP) controlMode = INVERSE_KINEMATICS;
		else controlMode = OPEN_LOOP;
	}
	
	if (controlMode == OPEN_LOOP) {
		if(IsKeyPressed(KEY_ZERO)) direction = direction? false : true;
		if(IsKeyDown(KEY_ONE)) buttonInput = 1; //X
		else if(IsKeyDown(KEY_TWO)) buttonInput = 2; //J2
		else if(IsKeyDown(KEY_THREE)) buttonInput = 3; //J3
		else if(IsKeyDown(KEY_FOUR)) buttonInput = 4; //J4
		else if(IsKeyDown(KEY_FIVE)) buttonInput = 5; //Pitch
		else if(IsKeyDown(KEY_SIX)) buttonInput = 6; //Roll Valkyrie
		else buttonInput = 0;
	} else if (controlMode == INVERSE_KINEMATICS) {
		if(IsKeyDown(KEY_UP)) WristPos.y += SPEED;
		if(IsKeyDown(KEY_DOWN)) WristPos.y -= SPEED;
		if(IsKeyDown(KEY_LEFT)) WristPos.x -= SPEED;
		if(IsKeyDown(KEY_RIGHT)) WristPos.x += SPEED;
		if(IsKeyDown(KEY_Z)) WristPos.z -= SPEED;
		if(IsKeyDown(KEY_C)) WristPos.z += SPEED;
		if(IsKeyDown(KEY_R)) q4.apparent -= (SPEED/8);
		if(IsKeyDown(KEY_F)) q4.apparent += (SPEED/8);
		if(IsKeyDown(KEY_E)) qP.apparent += (SPEED/8);
		if(IsKeyDown(KEY_Q)) qP.apparent -= (SPEED/8);
		if(IsKeyDown(KEY_V)) qV.apparent += (SPEED/8);
		if(IsKeyPressed(KEY_L)) lockMode = lockMode? false : true;
	} else if (controlMode == CLOSED_LOOP) {
		if(IsKeyDown(KEY_UP)) q2.target += (SPEED/8);
		if(IsKeyDown(KEY_DOWN)) q2.target -= (SPEED/8);
		if(IsKeyDown(KEY_LEFT)) q3.target -= (SPEED/8);
		if(IsKeyDown(KEY_RIGHT)) q3.target += (SPEED/8);
		if(IsKeyDown(KEY_Z)) q1.target -= (SPEED/8);
		if(IsKeyDown(KEY_C)) q1.target += (SPEED/8);
		if(IsKeyDown(KEY_R)) q4.target -= (SPEED/8);
		if(IsKeyDown(KEY_F)) q4.target += (SPEED/8);
		if(IsKeyDown(KEY_E)) qP.target += (SPEED/8);
		if(IsKeyDown(KEY_Q)) qP.target -= (SPEED/8);
		if(IsKeyDown(KEY_V)) qV.target += (SPEED/8);
	}
	
	if(IsKeyDown(KEY_P)) {
		WristPos.x = 10;
		WristPos.y = 0;
		WristPos.z = 0;
		q4.apparent = 0;
		qP.apparent = 0;
		qV.apparent = 0;
		underMode = false;
	}
}

void IK::CorrectAngles() {
	if (qP.apparent > 2*M_PI) qP.apparent -= 2*M_PI;
	if (qP.apparent < 0) qP.apparent += 2*M_PI;
	if (q4.apparent > 2*M_PI) q4.apparent -= 2*M_PI;
	if (q4.apparent < 0) q4.apparent += 2*M_PI;

	if (q4.motor > 2*M_PI) q4.motor -= 2*M_PI;
	if (q4.motor < 0) q4.motor += 2*M_PI;
	if (qP.motor > 2*M_PI) qP.motor -= 2*M_PI;
	if (qP.motor < 0) qP.motor += 2*M_PI;

	if (q4.motor > 2*M_PI) q4.target -= 2*M_PI;
	if (q4.motor < 0) q4.target += 2*M_PI;
	if (qP.motor > 2*M_PI) qP.target -= 2*M_PI;
	if (qP.motor < 0) qP.target += 2*M_PI;
}

void IK::CalculateIK() {

	//Calculate which solution to use
	if ((q3.motor >= 0) && (WristPos.x <= 0)) underMode = true;
	else if ((q3.motor <= 0) && (WristPos.x >= 0)) underMode = false;

	//Calculate motor angles using IK
	q1.target = WristPos.z;
	q3.target = acos((pow(WristPos.x,2)+pow(WristPos.y,2)-pow(J2_LENGTH,2)-pow(J3_LENGTH,2))/(2*J2_LENGTH*J3_LENGTH));
	if (sqrt(pow(WristPos.x,2) + pow(WristPos.y,2)) >= (J2_LENGTH+J3_LENGTH)) q3.target = 0;
	q2.target = underMode? atan2(WristPos.y, WristPos.x) - atan2(J3_LENGTH*sin(q3.target),J2_LENGTH+(J3_LENGTH*cos(q3.target))) : atan2(WristPos.y, WristPos.x) + atan2(J3_LENGTH*sin(q3.target),J2_LENGTH+(J3_LENGTH*cos(q3.target)));
	q3.target = underMode? q3.target : -q3.target;

	//Limit Joints
	// LimitJoint(q1);
	// LimitJoint(q2);
	// LimitJoint(q3);

	//Recalculate apparent angles
	// q2.apparent = q2.target;
	// q3.apparent = q2.target + q3.target;
	
	//Recalculate wrist position
	// WristPos.x = ((J2_LENGTH*cosf(q2.apparent))+(J3_LENGTH*cosf(q3.apparent)));
	// WristPos.y = ((J2_LENGTH*sinf(q2.apparent))+(J3_LENGTH*sinf(q3.apparent)));
	// WristPos.z = q1.target;

	//Calculate gripper position
	// GripperPos = {0,0,0}; //pixels
	// GripperPos = GripperPos * (MatrixTranslate(-VALK_LENGTH*INTOPIXELS, 0, 0) * ValkModel.transform);

	//////WRIST//////

	//Pitch
	
	// LimitJoint(qP);
	// qP.apparent = qP.target + q3.apparent*cosf(q4.target);

	//J4
	q4.target = q4.apparent;
	// LimitJoint(q4);
	// q4.apparent = q4.target;

	qP.target = qP.apparent - (q2.target + q3.target)*cosf(q4.target);

	//Valkyrie
	qV.target = qV.apparent - ((q2.target + q3.target)*sinf(q4.target))*sinf(qP.target);
}

bool IK::atFwdLim(joint q) {
	if (limsOverride) return false;
	if (q.RevLim > q.FwdLim) {
		return (q.motor >= q.FwdLim) && (q.motor <= (q.RevLim + q.FwdLim)/2);
	}
	return q.motor >= q.FwdLim;
}

bool IK::atRevLim(joint q) {
	if (limsOverride) return false;
	if (q.RevLim > q.FwdLim) {
		return (q.motor <= q.RevLim) && (q.motor >= (q.RevLim + q.FwdLim)/2);
	}
	return q.motor <= q.RevLim;
}

void IK::LimitJoint(joint &q) {
	CorrectAngles();
	if (atFwdLim(q)) q.target = q.motor = q.FwdLim;
	if (atRevLim(q)) q.target = q.motor = q.RevLim;
}

void IK::UpdateJoint(joint &q) {
	if (buttonInput == q.button) {
		direction? q.motor += (SPEED/16) : q.motor -= (SPEED/16); //Set decipercent
		q.target = q.motor; //update targets based on real motor
	} else if ((controlMode == CLOSED_LOOP) || (controlMode == INVERSE_KINEMATICS)) {
		q.motor = q.target; //SetAngle()
	}
	LimitJoint(q);
}

void IK::Update() {
	// CalcApparents();
	if (controlMode == INVERSE_KINEMATICS) {
		CalculateIK();
	}
	// CalcApparents();
	UpdateJoint(q1);
	UpdateJoint(q2);
	UpdateJoint(q3);
	UpdateJoint(q4);
	UpdateJoint(qP);
	UpdateJoint(qV);
	CalcApparents();
}

void IK::CalcApparents() {
	q1.apparent = q1.motor;
	q2.apparent = q2.motor;
	q3.apparent = q2.motor + q3.motor;

	WristPos.x = ((J2_LENGTH*cosf(q2.apparent))+(J3_LENGTH*cosf(q3.apparent)));
	WristPos.y = ((J2_LENGTH*sinf(q2.apparent))+(J3_LENGTH*sinf(q3.apparent)));
	WristPos.z = q1.apparent;

	GripperPos = {0,0,0}; //pixels
	GripperPos = GripperPos * (MatrixTranslate(-VALK_LENGTH*INTOPIXELS, 0, 0) * ValkModel.transform);

	qP.apparent = qP.motor + q3.apparent*cosf(q4.motor);
	q4.apparent = q4.motor;
}

/*
CURRENT ISSUES:
	Pitch slides with J4, J3, and J2???


*/