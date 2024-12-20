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

#define J1_FWD_LIM 6.3
#define J1_REV_LIM -6.3

#define J2_FWD_LIM 164
#define J2_REV_LIM -54

#define J3_FWD_LIM 90
#define J3_REV_LIM -116.8

#define PITCH_FWD_LIM 355
#define PITCH_REV_LIM 360

#define J4_FWD_LIM 350
#define J4_REV_LIM 360

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
            float target;
            int decipercent;
            float FwdLim;
            float RevLim;
            int button;
        };

        joint q1, q2, q3, q4, qP, qV;

        enum ControlMode {
            OPEN_LOOP,
            CLOSED_LOOP,
            INVERSE_KINEMATICS
        };

        bool underMode, lockMode, limsOverride, direction;
        ControlMode controlMode;
        int buttonInput;

    public:
        IK() {
            WristPos = {10.0f,0.0f, 0.0f};
            GripperPos = {0, 0, 0};

            q1 = {0, 0, 0, 0, J1_FWD_LIM, J1_REV_LIM, 1};
            q2 = {0, 0, 0, 0, J2_FWD_LIM*DEG2RAD, J2_REV_LIM*DEG2RAD, 2};
            q3 = {0, 0, 0, 0, J3_FWD_LIM*DEG2RAD, J3_REV_LIM*DEG2RAD, 3};
            q4 = {0, 0, 0, 0, J4_FWD_LIM*DEG2RAD, J4_REV_LIM*DEG2RAD, 4};
            qP = {0, 0, 0, 0, PITCH_FWD_LIM*DEG2RAD, PITCH_REV_LIM*DEG2RAD, 5};
            qV = {0, 0, 0, 0, 360, 0, 6};
 
            underMode = lockMode = 0;
            limsOverride = false;
            direction = false;
            buttonInput = 0;
            ControlMode controlMode = OPEN_LOOP;
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
        void Keyboard();
        void CorrectAngles();
        void CalculateIK();
        bool atFwdLim(joint q);
        bool atRevLim(joint q);
        void LimitJoint(joint &q);
        void UpdateJoint(joint &q);
        void Update();
        void CalcApparents();
};

#endif