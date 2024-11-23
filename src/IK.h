#ifndef IK_H
#define IK_H

using namespace std;

#include <math.h>
#include "raylib.h"
#include "raymath.h"
#include "resource_dir.h"

#define SPEED 0.2f

#define J2_LENGTH 18
#define J3_LENGTH 18.5
#define WRIST_RAD 2.887499685
#define SHOULDER_LENGTH 7.328739921
#define VALK_LENGTH 6.24943834646

#define INTOPIXELS 12.7
#define PIXELSTOIN (1/12.7)

class IK {
    private:
        Model J1Model;
        Model J4Model;
        Model J2Model;
        Model PitchModel;
        Model ValkModel;
        Model SolModel;

        Vector3 WristPos;
        Vector3 GripperPos;

        struct angle {
            float motor;
            float apparent;
        };

        angle q2, q3, q4, qP, qV;

        bool underMode, lockMode;

    public:
        IK() {
            WristPos = {10.0f,0.0f, 0.0f};
            GripperPos = {0, 0, 0};

            q2 = q3 = q4 = qP = qV = {0, 0};
            underMode = lockMode = 0;
            J1Model = LoadModel("J1Model.obj");
            J2Model = LoadModel("J2Model.obj");
            J4Model = LoadModel("J4Model.obj");
            PitchModel = LoadModel("PitchModel.obj");
            ValkModel = LoadModel("ValkModel.obj");
            SolModel = LoadModel("SolModel.obj");
        }

        void Draw();
        void Unload();
        void Transform();
        void CalcWristAngles();
        void Keyboard();
        void CorrectAngles();
        void CalcLinearMovement();
};

#endif