#include "IK.h"

int main ()
{
	SetConfigFlags(FLAG_VSYNC_HINT | FLAG_WINDOW_HIGHDPI);
	InitWindow(1280, 720, "ArmSimulation");
	SearchAndSetResourceDir("resources");
	SetTargetFPS(60);

	Camera camera;
	camera.position = (Vector3){ -200.0f, 100.0f, -200.0f };
	camera.target = (Vector3){ -200.0f, 100.0f, 0.0f };
	camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
	camera.fovy = 90.0f;
	camera.projection = CAMERA_PERSPECTIVE;
	DisableCursor();

	IK ik;
	
	while (!WindowShouldClose())
	{
		UpdateCamera(&camera, CAMERA_FREE);
		ClearBackground(WHITE);

		ik.Transform();
		ik.Keyboard();
		ik.CorrectAngles();
		ik.Update();

		BeginDrawing();
		BeginMode3D(camera);
		ik.Draw();
	}

	ik.Unload();
	CloseWindow();
	return 0;
}

/*
TODO:
	Features:
		Display angles and position
		Shader?
		Path of motion for keyboard typing
		Software limits
		Add limits (collision detection?)
			Need to add x axis and rover?
		Set pos mode

	Issues:
		Slides on range of motion
		Lock mode
*/