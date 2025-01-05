#include "IK.h"

void IK::Draw() 
{
	DrawModel(J2.model, (Vector3){0.0f, 0.0f, 0.0f }, 1.0f, RED);		
	DrawModel(J3.model, (Vector3){0, 0, 0}, 1.0f, BLUE);
	DrawModel(J4.model, (Vector3){0, 0, 0}, 1.0f, ORANGE);
	DrawModel(Pitch.model, (Vector3){0, 0, 0}, 1.0f, PURPLE);
	DrawModel(Valkyrie.model, (Vector3){0, 0, 0}, 1.0f, GREEN);
	DrawModel(SolenoidModel, (Vector3){0, 0, 0}, 1.0f, MAROON);
	DrawSphere(GripperPos, 2, MAROON);
	DrawGrid(100, 10.0f);

	EndMode3D();

	DrawText("Arm Simulation", 5,5,20,BLACK);
	DrawText("CM: ", 5,700,20,BLACK);
	if (currentMode == OPEN_LOOP) DrawText("O", 150,700,20,BLACK);
	else if (currentMode == CLOSED_LOOP) DrawText("C", 150,700,20,BLACK);
	else DrawText("IK", 150,700,20,BLACK);
	DrawText("X: ", 5,650,20,BLACK);
	DrawText(TextFormat("%.2f", WristPos.x), 40,650,20,BLACK);
	DrawText("Y: ", 150,650,20,BLACK);
	DrawText(TextFormat("%.2f", WristPos.y), 190,650,20,BLACK);
	DrawText("PT: ", 5,600,20,BLACK);
	DrawText(TextFormat("%.2f", Pitch.qTarget), 60,600,20,BLACK);
	// DrawText("Valkyriet: ", 150,600,20,BLACK);
	// DrawText(TextFormat("%.2f", RAD2DEG*Valkyrie.qTarget), 210,600,20,BLACK);

	EndDrawing();
}

void IK::Unload() 
{
	UnloadModel(J2.model);
	UnloadModel(J3.model);
	UnloadModel(J4.model);
	UnloadModel(Pitch.model);
	UnloadModel(Valkyrie.model);
	UnloadModel(SolenoidModel);
}

void IK::TransformArm() 
{
	J2.model.transform = MatrixRotateZ(-J2.qMotor*DEG2RAD) * MatrixTranslate(0, 0, INTOPIXELS*J1.qMotor);
	// J2.transf = Rotate(0,0, -J2.qMotor*DEG2RAD) * Translate(0,0, float(INTOPIXELS*J1.qMotor));
	// J2.model.transform.m0 = J2.transf.m0;
	// J2.model.transform.m1 = J2.transf.m1;
	// J2.model.transform.m2 = J2.transf.m2;
	// J2.model.transform.m3 = J2.transf.m3;
	// J2.model.transform.m4 = J2.transf.m4;
	// J2.model.transform.m5 = J2.transf.m5;
	// J2.model.transform.m6 = J2.transf.m6;
	// J2.model.transform.m7 = J2.transf.m7;
	// J2.model.transform.m8 = J2.transf.m8;
	// J2.model.transform.m9 = J2.transf.m9;
	// J2.model.transform.m10 = J2.transf.m10;
	// J2.model.transform.m11 = J2.transf.m11;
	// J2.model.transform.m12 = J2.transf.m12;
	// J2.model.transform.m13 = J2.transf.m13;
	// J2.model.transform.m14 = J2.transf.m14;
	// J2.model.transform.m15 = J2.transf.m15;

	J3.model.transform = MatrixRotateZ(-J3.qMotor*DEG2RAD) * MatrixTranslate(-J2_LENGTH*INTOPIXELS, 0, 0) * J2.model.transform;
	J4.model.transform = MatrixRotateX((-J4.qMotor+90)*DEG2RAD) * MatrixTranslate(-SHOULDER_LENGTH*INTOPIXELS, 0, 0) * J3.model.transform;
	Pitch.model.transform = MatrixRotateZ(-Pitch.qMotor*DEG2RAD) * MatrixTranslate(-(J3_LENGTH-SHOULDER_LENGTH)*INTOPIXELS, 0, 0) * J4.model.transform;
	Valkyrie.model.transform = MatrixRotateX(-Valkyrie.qMotor*DEG2RAD) * MatrixTranslate(-WRIST_RAD*INTOPIXELS, 0, 0) * Pitch.model.transform;
	SolenoidModel.transform = MatrixRotateY(M_PI) * MatrixTranslate(WRIST_RAD*INTOPIXELS, 0, 0) * Pitch.model.transform;
}

void IK::Keyboard() 
{
	if (IsKeyPressed(KEY_M)) {
		if (currentMode == OPEN_LOOP) currentMode = CLOSED_LOOP;
		else if (currentMode == CLOSED_LOOP) currentMode = INVERSE_KINEMATICS;
		else currentMode = OPEN_LOOP;
	}
	
	if (currentMode == OPEN_LOOP) {
		lockMode = false;
		if(IsKeyPressed(KEY_ZERO)) direction = direction? false : true;
		if(IsKeyDown(KEY_ONE)) buttonInput = 1; //X
		else if(IsKeyDown(KEY_TWO)) buttonInput = 2; //J2
		else if(IsKeyDown(KEY_THREE)) buttonInput = 3; //J3
		else if(IsKeyDown(KEY_FOUR)) buttonInput = 4; //J4
		else if(IsKeyDown(KEY_FIVE)) buttonInput = 5; //Pitch
		else if(IsKeyDown(KEY_SIX)) buttonInput = 6; //Roll Valkyrie
		else buttonInput = 0;
	} else if (currentMode == INVERSE_KINEMATICS) {
		if(IsKeyDown(KEY_UP)) WristPos.y += SPEED;
		if(IsKeyDown(KEY_DOWN)) WristPos.y -= SPEED;
		if(IsKeyDown(KEY_LEFT)) WristPos.x -= SPEED;
		if(IsKeyDown(KEY_RIGHT)) WristPos.x += SPEED;		
		if(IsKeyDown(KEY_Z)) WristPos.z -= SPEED;
		if(IsKeyDown(KEY_C)) WristPos.z += SPEED;
		if(IsKeyDown(KEY_R)) wrist.j4 += (SPEED*SPD_MOD4);
		if(IsKeyDown(KEY_F)) wrist.j4 -= (SPEED*SPD_MOD4);
		if(IsKeyDown(KEY_E)) wrist.pitch += (SPEED*SPD_MOD4);
		if(IsKeyDown(KEY_Q)) wrist.pitch -= (SPEED*SPD_MOD4);
		if(IsKeyDown(KEY_V)) wrist.valk += (SPEED*SPD_MOD4);
		if(IsKeyPressed(KEY_L)) lockMode = lockMode? false : true;
	} else if (currentMode == CLOSED_LOOP) {
		lockMode = false;
		if(IsKeyDown(KEY_UP)) J2.qTarget += (SPEED*SPD_MOD2);
		if(IsKeyDown(KEY_DOWN)) J2.qTarget -= (SPEED*SPD_MOD2);
		if(IsKeyDown(KEY_LEFT)) J3.qTarget -= (SPEED*SPD_MOD2);
		if(IsKeyDown(KEY_RIGHT)) J3.qTarget += (SPEED*SPD_MOD2);
		if(IsKeyDown(KEY_Z)) J1.qTarget -= (SPEED*SPD_MOD2);
		if(IsKeyDown(KEY_C)) J1.qTarget += (SPEED*SPD_MOD2);
		if(IsKeyDown(KEY_R)) J4.qTarget -= (SPEED*SPD_MOD4);
		if(IsKeyDown(KEY_F)) J4.qTarget += (SPEED*SPD_MOD4);
		if(IsKeyDown(KEY_E)) Pitch.qTarget += (SPEED*SPD_MOD4);
		if(IsKeyDown(KEY_Q)) Pitch.qTarget -= (SPEED*SPD_MOD4);
		if(IsKeyDown(KEY_V)) Valkyrie.qTarget += (SPEED*SPD_MOD4);
	}
	
	if(IsKeyDown(KEY_P)) {
		WristPos.x = 10;
		WristPos.y = 0;
		WristPos.z = 0;
		wrist.j4 = 0;
		wrist.pitch = 0;
		wrist.valk = 0;
		underMode = false;
	}
}

void IK::CalculateIK() 
{

	//Calculate which solution to use
	underMode = (J3.qMotor > 0)? true : false;

	//Limit J3 based on which solution is used
	if (underMode) {
		J3.FwdLim = J3_POS_LIM;
		J3.RevLim = J3_MID_LIM;
	} else {
		J3.FwdLim = (-1)*J3_MID_LIM;
		J3.RevLim = J3_NEG_LIM;
	}

	//Calculate target angles using IK
	J1.qTarget = WristPos.z;
	J3.qTarget = RAD2DEG*acos((pow(WristPos.x,2)+pow(WristPos.y,2)-pow(J2_LENGTH,2)-pow(J3_LENGTH,2))/(2*J2_LENGTH*J3_LENGTH));

	if (underMode) J2.qTarget = RAD2DEG*(atan2(WristPos.y, WristPos.x) - atan2(J3_LENGTH*sin(J3.qTarget*DEG2RAD),J2_LENGTH+(J3_LENGTH*cos(J3.qTarget*DEG2RAD))));
	else J2.qTarget = RAD2DEG*(atan2(WristPos.y, WristPos.x) + atan2(J3_LENGTH*sin(J3.qTarget*DEG2RAD),J2_LENGTH+(J3_LENGTH*cos(J3.qTarget*DEG2RAD))));
	
	J3.qTarget = underMode? J3.qTarget : -J3.qTarget;

	//Calculate spherical wrist
	J4.qTarget = 90; //Lock J4
	Pitch.qTarget = wrist.pitch - (J2.qTarget + J3.qTarget);
	Valkyrie.qTarget = wrist.valk;

	// Check if calculated angle is invalid and limit movement
	if (isOutsideTargetRange(J1) || isOutsideTargetRange(J2) || isOutsideTargetRange(J3) || isOutsideTargetRange(Pitch)) CalculateApparents();

}

bool IK::atFwdLim(joint J) 
{
	if (limsOverride) return false;
	if (J.RevLim > J.FwdLim) {
		return (J.qMotor >= J.FwdLim) && (J.qMotor <= (J.RevLim + J.FwdLim)/2);
	}
	return J.qMotor >= J.FwdLim;
}

bool IK::atRevLim(joint J) 
{
	if (limsOverride) return false;
	if (J.RevLim > J.FwdLim) {
		return (J.qMotor <= J.RevLim) && (J.qMotor >= (J.RevLim + J.FwdLim)/2);
	}
	return J.qMotor <= J.RevLim;
}

bool IK::isOutsideTargetRange(joint J) 
{
	if (J.qTarget > J.FwdLim) return true;
	if (J.qTarget < J.RevLim) return true;
	return false;
}

void IK::LimitJoint(joint &J) 
{
	if (atFwdLim(J)) J.qMotor = J.FwdLim;
	if (atRevLim(J)) J.qMotor = J.RevLim;
}

void IK::UpdateJoint(joint &J) 
{
	if (currentMode == CLOSED_LOOP) { //Limit closed loop target angles
		if (J.qTarget > J.FwdLim) J.qTarget = J.FwdLim;
		else if (J.qTarget < J.RevLim) J.qTarget = J.RevLim;
	}

	if (buttonInput == J.button) {
		direction? J.qMotor += (SPEED*2) : J.qMotor -= (SPEED*2); //Set decipercent
	} else if ((currentMode == CLOSED_LOOP) || (currentMode == INVERSE_KINEMATICS)) {
		J.qMotor = J.qTarget; //SetAngle()
	}
	LimitJoint(J);
}

void IK::Update() 
{
	if (((currentMode == CLOSED_LOOP) || (currentMode == INVERSE_KINEMATICS)) && (currentMode != prevMode)) CalculateApparents();
	if (currentMode == INVERSE_KINEMATICS) {
		CalculateIK();
	} else {
		J3.FwdLim = J3_POS_LIM;
		J3.RevLim = J3_NEG_LIM;
	}
	UpdateJoint(J1);
	UpdateJoint(J2);
	UpdateJoint(J3);
	UpdateJoint(J4);
	UpdateJoint(Pitch);
	UpdateJoint(Valkyrie);

	prevMode = currentMode;

	GripperPos = {0,0,0}; //pixels
	GripperPos = GripperPos * (MatrixTranslate(-VALK_LENGTH*INTOPIXELS, 0, 0) * Valkyrie.model.transform);
}

void IK::CalculateApparents() 
{
	WristPos.x = ((J2_LENGTH*cosf(J2.qMotor*DEG2RAD))+(J3_LENGTH*cosf((J2.qMotor + J3.qMotor)*DEG2RAD))); //Use matrix library
	WristPos.y = ((J2_LENGTH*sinf(J2.qMotor*DEG2RAD))+(J3_LENGTH*sinf((J2.qMotor + J3.qMotor)*DEG2RAD)));
	WristPos.z = J1.qMotor;

	wrist.pitch = Pitch.qMotor + J2.qMotor + J3.qMotor;
	wrist.j4 = J4.qMotor;
	wrist.valk = Valkyrie.qMotor;

	J1.qTarget = J1.qMotor;
	J2.qTarget = J2.qMotor;
	J3.qTarget = J3.qMotor;
	J4.qTarget = J4.qMotor;
	Pitch.qTarget = Pitch.qMotor;
	Valkyrie.qTarget = Valkyrie.qMotor;
}
/*
Questions:
Set target ONCE when entering closed mode or IK mode
	change in real software too, only relevant in closed loop and IK
*/
