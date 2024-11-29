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

#define J1_FWD_LIM 9
#define J1_REV_LIM -9

#define J2_FWD_LIM 180
#define J2_REV_LIM -55

#define J3_FWD_LIM 115
#define J3_REV_LIM -135

#define PITCH_FWD_LIM 350
#define PITCH_REV_LIM 10

#define J4_FWD_LIM 350
#define J4_REV_LIM 355

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

        struct joint {
            float motor;
            float apparent;
            float FwdLim;
            float RevLim;
        };

        joint q1, q2, q3, q4, qP, qV;

        bool underMode, lockMode;

    public:
        IK() {
            WristPos = {10.0f,0.0f, 0.0f};
            GripperPos = {0, 0, 0};

            q1 = {0, 0, J1_FWD_LIM, J1_REV_LIM};
            q2 = {0, 0, J2_FWD_LIM*DEG2RAD, J2_REV_LIM*DEG2RAD};
            q3 = {-20*DEG2RAD, 0, J3_FWD_LIM*DEG2RAD, J3_REV_LIM*DEG2RAD};
            q4 = {0, 0, J4_FWD_LIM*DEG2RAD, J4_REV_LIM*DEG2RAD};
            qP = {0, 0, PITCH_FWD_LIM*DEG2RAD, PITCH_REV_LIM*DEG2RAD};
 
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
        bool atFwdLim(joint q);
        bool atRevLim(joint q);
};

#endif