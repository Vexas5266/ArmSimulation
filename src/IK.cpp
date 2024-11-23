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
	DrawText("q1: ", 5,700,20,BLACK);
	DrawText(TextFormat("%.2f",(180/M_PI)*q2.motor), 40,700,20,BLACK);
	DrawText("q2: ", 150,700,20,BLACK);
	DrawText(TextFormat("%.2f",(180/M_PI)*q3.motor), 190,700,20,BLACK);
	DrawText("X: ", 5,650,20,BLACK);
	DrawText(TextFormat("%.2f", WristPos.x), 40,650,20,BLACK);
	DrawText("Y: ", 150,650,20,BLACK);
	DrawText(TextFormat("%.2f", WristPos.y), 190,650,20,BLACK);
	DrawText("q4: ", 5,600,20,BLACK);
	DrawText(TextFormat("%.2f", RAD2DEG*q4.motor), 60,600,20,BLACK);
	DrawText("qP: ", 150,600,20,BLACK);
	DrawText(TextFormat("%.2f", RAD2DEG*qP.motor), 210,600,20,BLACK);
	// DrawText(": ", 150,600,20,BLACK);
	// DrawText(TextFormat("%.2f", RAD2DEG*qP), 210,600,20,BLACK);

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
	J1Model.transform = MatrixRotateZ(-q2.motor) * MatrixTranslate(0, 0, INTOPIXELS*WristPos.z);
	J2Model.transform = MatrixRotateZ(-q3.motor) * MatrixTranslate(-J2_LENGTH*INTOPIXELS, 0, 0) * J1Model.transform;
	J4Model.transform = MatrixRotateX(-q4.motor) * MatrixTranslate(-SHOULDER_LENGTH*INTOPIXELS, 0, 0) * J2Model.transform;
	PitchModel.transform = MatrixRotateZ(-qP.motor) * MatrixTranslate(-(J3_LENGTH-SHOULDER_LENGTH)*INTOPIXELS, 0, 0) * J4Model.transform;
	ValkModel.transform = MatrixRotateX(-qV.motor) * MatrixTranslate(-WRIST_RAD*INTOPIXELS, 0, 0) * PitchModel.transform;
	SolModel.transform = MatrixRotateY(M_PI) * MatrixTranslate(WRIST_RAD*INTOPIXELS, 0, 0) * PitchModel.transform;
}

void IK::CalcWristAngles() {
	qP.motor = qP.apparent - q3.apparent*cosf(q4.motor);
	q4.motor = q4.apparent;
	qV.motor = qV.apparent - (q3.apparent*sinf(q4.motor))*sinf(qP.motor);
}

void IK::Keyboard() {
	if(IsKeyDown(KEY_UP)) WristPos.y += SPEED;
	if(IsKeyDown(KEY_DOWN)) WristPos.y -= SPEED;
	if(IsKeyDown(KEY_LEFT)) WristPos.x -= SPEED;
	if(IsKeyDown(KEY_RIGHT)) WristPos.x += SPEED;
	if(IsKeyDown(KEY_Z)) WristPos.z -= SPEED;
	if(IsKeyDown(KEY_C)) WristPos.z += SPEED;
	if(IsKeyDown(KEY_R)) q4.apparent -= (SPEED/8);
	if(IsKeyDown(KEY_F)) q4.apparent += (SPEED/8);
	if(IsKeyDown(KEY_Q)) qP.apparent -= (SPEED/8);
	if(IsKeyDown(KEY_E)) qP.apparent += (SPEED/8);
	if(IsKeyDown(KEY_V)) qV.apparent += (SPEED/8);

	if (lockMode) {
		if(IsKeyPressed(KEY_L)) lockMode = false;
	} else {
		if(IsKeyPressed(KEY_L)) lockMode = true;
	}

	if (underMode) {
		if(IsKeyPressed(KEY_U)) underMode = false;
	} else {
		if(IsKeyPressed(KEY_U)) underMode = true;
	}
	
	if(IsKeyDown(KEY_P)) {
		WristPos.x = 10;
		WristPos.y = 0;
		WristPos.z = 0;
		q4.apparent = 0;
		qP.apparent = 0;
		qV.apparent = 0;
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
}

void IK::CalcLinearMovement() {
	if (underMode) {
		q3.motor = acos((pow(WristPos.x,2)+pow(WristPos.y,2)-pow(J2_LENGTH,2)-pow(J3_LENGTH,2))/(2*J2_LENGTH*J3_LENGTH));
		q2.motor = atan2(WristPos.y, WristPos.x) - atan2(J3_LENGTH*sin(q3.motor),J2_LENGTH+(J3_LENGTH*cos(q3.motor)));
		q3.apparent = q2.motor + q3.motor;
	} else {
		q3.motor = acos((pow(WristPos.x,2)+pow(WristPos.y,2)-pow(J2_LENGTH,2)-pow(J3_LENGTH,2))/(2*J2_LENGTH*J3_LENGTH));
		q2.motor = atan2(WristPos.y, WristPos.x) + atan2(J3_LENGTH*sin(q3.motor),J2_LENGTH+(J3_LENGTH*cos(q3.motor)));
		q3.apparent = q2.motor - q3.motor;
		q3.motor = -q3.motor;
	}
	if (lockMode) {
		WristPos.x = -(PIXELSTOIN*GripperPos.x + (VALK_LENGTH + WRIST_RAD)*cosf(qP.apparent));
		WristPos.y = (PIXELSTOIN*GripperPos.y - (VALK_LENGTH+WRIST_RAD)*sinf(q3.apparent)*cosf(qP.motor)*sinf(q4.apparent)*sinf(q4.apparent) - ((VALK_LENGTH + WRIST_RAD)*sinf(qP.apparent)*cosf(q4.apparent)));
		WristPos.z = (PIXELSTOIN*GripperPos.z - (VALK_LENGTH + WRIST_RAD)*sinf(q4.apparent)*sinf(qP.motor-M_PI));
	} else {
		GripperPos = {0,0,0};
		GripperPos = GripperPos * (MatrixTranslate(-VALK_LENGTH*INTOPIXELS, 0, 0) * ValkModel.transform);
	}
}