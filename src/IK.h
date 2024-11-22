using namespace std;

//#include <iostream>
#include <math.h>
//#include <vector>

struct coords {
	float x, y, z;
};

coords target{10.0f,0.0f, 0.0f};
Vector3 LockOnPos = {0, 0, 0};

#define SPEED 0.2f

#define J1_LENGTH 18
#define J2_LENGTH 18.5
#define WRIST_RAD 2.887499685
#define SHOULDER_LENGTH 7.328739921
#define VALK_LENGTH 6.24943834646

#define INTOPIXELS 12.7
#define PIXELSTOIN (1/12.7)

float speed, q1, q2, qT, q4, qP, qV, q4Target, qPTarget, qVTarget;
bool underMode, lockMode=0;