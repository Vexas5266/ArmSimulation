#include "raylib.h"
#include "raymath.h"
#include "resource_dir.h"	// utility header for SearchAndSetResourceDir
#include "IK.h"

using namespace std;

int main ()
{
	SetConfigFlags(FLAG_VSYNC_HINT | FLAG_WINDOW_HIGHDPI);
	InitWindow(1280, 720, "ArmSimulation");
	SearchAndSetResourceDir("resources");
	SetTargetFPS(60);

	Camera camera = { 0 };;
	camera.position = (Vector3){ -200.0f, 100.0f, -200.0f };
    camera.target = (Vector3){ -200.0f, 100.0f, 0.0f };
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
    camera.fovy = 90.0f;
    camera.projection = CAMERA_PERSPECTIVE;
	DisableCursor();

	Model J1Model = LoadModel("J1Model.obj");
	Model J2Model = LoadModel("J2Model.obj");
	Model J4Model = LoadModel("J4Model.obj");
	Model PitchModel = LoadModel("PitchModel.obj");
	Model ValkModel = LoadModel("ValkModel.obj");
	Model SolModel = LoadModel("SolModel.obj");

	//Shader shader = LoadShader("shader.vs", "shader.fs");
	// J1Model.materials[0].shader = shader;
	// J2Model.materials[0].shader = shader;
	// J4Model.materials[0].shader = shader;
	// PitchModel.materials[0].shader = shader;
	
	while (!WindowShouldClose())
	{
		BeginDrawing();
		UpdateCamera(&camera, CAMERA_FREE);
		ClearBackground(WHITE);

		//Update model rotations and translations like irl
		J1Model.transform = MatrixRotateZ(-q1) * MatrixTranslate(0, 0, INTOPIXELS*target.z);
		J2Model.transform = MatrixRotateZ(-q2) * MatrixTranslate(-J1_LENGTH*INTOPIXELS, 0, 0) * J1Model.transform;
		J4Model.transform = MatrixRotateX(-q4) * MatrixTranslate(-SHOULDER_LENGTH*INTOPIXELS, 0, 0) * J2Model.transform;
		PitchModel.transform = MatrixRotateZ(-qP) * MatrixTranslate(-(J2_LENGTH-SHOULDER_LENGTH)*INTOPIXELS, 0, 0) * J4Model.transform;
		ValkModel.transform = MatrixRotateX(-qV) * MatrixTranslate(-WRIST_RAD*INTOPIXELS, 0, 0) * PitchModel.transform;
		SolModel.transform = MatrixRotateY(M_PI) * MatrixTranslate(WRIST_RAD*INTOPIXELS, 0, 0) * PitchModel.transform;

		//Spherical Wrist
		qP = qPTarget - qT*cosf(q4);
		q4 = q4Target;
		qV = qVTarget - (qT*sinf(q4))*sinf(qP);

		//Keyboard
		if (lockMode) {
			
			if(IsKeyPressed(KEY_L)) lockMode = false;
			
			if(IsKeyDown(KEY_R)) q4Target -= (SPEED/8);
			if(IsKeyDown(KEY_F)) q4Target += (SPEED/8);
			if(IsKeyDown(KEY_Q)) qPTarget -= (SPEED/8);
			if(IsKeyDown(KEY_E)) qPTarget += (SPEED/8);
			if(IsKeyDown(KEY_V)) qVTarget += (SPEED/8);

			target.x = -(PIXELSTOIN*LockOnPos.x + (VALK_LENGTH + WRIST_RAD)*cosf(qPTarget));
			target.y = (PIXELSTOIN*LockOnPos.y - (VALK_LENGTH+WRIST_RAD)*sinf(qT)*cosf(qP)*sinf(q4Target)*sinf(q4Target) - ((VALK_LENGTH + WRIST_RAD)*sinf(qPTarget)*cosf(q4Target)));
			target.z = (PIXELSTOIN*LockOnPos.z - (VALK_LENGTH + WRIST_RAD)*sinf(q4Target)*sinf(qP-M_PI));

		} else {
			LockOnPos = {0,0,0};
			LockOnPos = LockOnPos * (MatrixTranslate(-VALK_LENGTH*INTOPIXELS, 0, 0) * ValkModel.transform);

			if(IsKeyDown(KEY_UP)) target.y += SPEED;
			if(IsKeyDown(KEY_DOWN)) target.y -= SPEED;
			if(IsKeyDown(KEY_LEFT)) target.x -= SPEED;
			if(IsKeyDown(KEY_RIGHT)) target.x += SPEED;
			if(IsKeyDown(KEY_Z)) target.z -= SPEED;
			if(IsKeyDown(KEY_C)) target.z += SPEED;

			if(IsKeyDown(KEY_R)) q4Target -= (SPEED/4);
			if(IsKeyDown(KEY_F)) q4Target += (SPEED/4);
			if(IsKeyDown(KEY_Q)) qPTarget -= (SPEED/4);
			if(IsKeyDown(KEY_E)) qPTarget += (SPEED/4);
			if(IsKeyDown(KEY_V)) qVTarget += (SPEED/4);

			if(IsKeyPressed(KEY_L)) lockMode = true;
		}

		if(IsKeyDown(KEY_P)) {
			target.x = 10;
			target.y = 0;
			target.z = 0;
			q4Target = 0;
			qPTarget = 0;
			qVTarget = 0;
		}

		if (qPTarget > 2*M_PI) qPTarget -= 2*M_PI;
		if (qPTarget < 0) qPTarget += 2*M_PI;
		if (q4Target > 2*M_PI) q4Target -= 2*M_PI;
		if (q4Target < 0) q4Target += 2*M_PI;

		if (q4 > 2*M_PI) q4 -= 2*M_PI;
		if (q4 < 0) q4 += 2*M_PI;
		if (qP > 2*M_PI) qP -= 2*M_PI;
		if (qP < 0) qP += 2*M_PI;

		//Linear Movement
		underMode = 0;
		if (underMode) {
			q2 = acos((pow(target.x,2)+pow(target.y,2)-pow(J1_LENGTH,2)-pow(J2_LENGTH,2))/(2*J1_LENGTH*J2_LENGTH));
			q1 = atan2(target.y, target.x) - atan2(J2_LENGTH*sin(q2),J1_LENGTH+(J2_LENGTH*cos(q2)));
			qT = q1 + q2;
		} else {
			q2 = acos((pow(target.x,2)+pow(target.y,2)-pow(J1_LENGTH,2)-pow(J2_LENGTH,2))/(2*J1_LENGTH*J2_LENGTH));
			q1 = atan2(target.y, target.x) + atan2(J2_LENGTH*sin(q2),J1_LENGTH+(J2_LENGTH*cos(q2)));
			qT = q1 - q2;
			q2 = -q2;
		}

		BeginMode3D(camera);
		// BeginShaderMode(shader);

		DrawModel(J1Model, (Vector3){0.0f, 0.0f, 0.0f }, 1.0f, RED);		
		DrawModel(J2Model, (Vector3){0, 0, 0}, 1.0f, BLUE);
        DrawModel(J4Model, (Vector3){0, 0, 0}, 1.0f, ORANGE);
		DrawModel(PitchModel, (Vector3){0, 0, 0}, 1.0f, PURPLE);
		DrawModel(ValkModel, (Vector3){0, 0, 0}, 1.0f, GREEN);
		DrawModel(SolModel, (Vector3){0, 0, 0}, 1.0f, MAROON);
		DrawSphere(LockOnPos, 2, MAROON);
		DrawGrid(100, 10.0f);

		EndShaderMode();
		EndMode3D();

		DrawText("Arm Simulation", 5,5,20,BLACK);
		DrawText("q1: ", 5,700,20,BLACK);
		DrawText(TextFormat("%.2f",(180/M_PI)*q1), 40,700,20,BLACK);
		DrawText("q2: ", 150,700,20,BLACK);
		DrawText(TextFormat("%.2f",(180/M_PI)*q2), 190,700,20,BLACK);
		DrawText("X: ", 5,650,20,BLACK);
		DrawText(TextFormat("%.2f", target.x), 40,650,20,BLACK);
		DrawText("Y: ", 150,650,20,BLACK);
		DrawText(TextFormat("%.2f", target.y), 190,650,20,BLACK);
		DrawText("q4: ", 5,600,20,BLACK);
		DrawText(TextFormat("%.2f", RAD2DEG*q4), 60,600,20,BLACK);
		DrawText("qP: ", 150,600,20,BLACK);
		DrawText(TextFormat("%.2f", RAD2DEG*qP), 210,600,20,BLACK);
		// DrawText(": ", 150,600,20,BLACK);
		// DrawText(TextFormat("%.2f", RAD2DEG*qP), 210,600,20,BLACK);

		EndDrawing();
	}

	UnloadModel(J1Model);
	UnloadModel(J2Model);
	UnloadModel(J4Model);
	UnloadModel(PitchModel);
	UnloadModel(ValkModel);
	UnloadModel(SolModel);
	CloseWindow();
	return 0;
}

/*
TODO:
Get cartesian coords from angles, not target
Fix angles to be more clear, have two separate: target and actual
xy based on gripper location
Separate joint drawing angles and position to be more clear
Display angles and position
Shader?
Path of motion for keyboard typing
Software limits
Add limits (collision detection?)
	Need to add x axis and rover?
2 modes: lock on point/unlock
Set pos mode
*/