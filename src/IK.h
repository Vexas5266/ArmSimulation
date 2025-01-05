#ifndef IK_H
#define IK_H

#include <math.h>
#include "raylib.h"
#include "raymath.h"
#include "resource_dir.h"
#include "InverseKinematics.h"
#include "RoveMatrix.h"

#define SPEED 0.2f
#define SPD_MOD2 2.0f
#define SPD_MOD4 4.0f


#define J2_LENGTH 18
#define J3_LENGTH 18.5
#define WRIST_RAD 2.887499685
#define SHOULDER_LENGTH 7.328739921
#define VALK_LENGTH 6.24943834646

#define J1_FWD_LIM 6.3
#define J1_REV_LIM -6.3

#define J2_FWD_LIM 164
#define J2_REV_LIM -54

#define J3_POS_LIM 90
#define J3_NEG_LIM -116.8
#define J3_MID_LIM 15

#define PITCH_FWD_LIM 350
#define PITCH_REV_LIM 10

#define J4_FWD_LIM 90
#define J4_REV_LIM -260

#define INTOPIXELS 12.7
#define PIXELSTOIN (1/12.7)

class IK {
    private:

        Vector WristPos;
        Vector GripperPos;

        struct joint {
            float qMotor; //in degrees
            float qTarget; //in degrees
            float FwdLim; //in degrees
            float RevLim; //in degrees
            int button;
            TransfMatrix transf;
            Model model;
        };

        struct Wrist {
            float j4;
            float pitch;
            float valk;
        };

        joint J1, J2, J3, J4, Pitch, Valkyrie, Solenoid;
        Wrist wrist;

        enum ControlMode {
            OPEN_LOOP,
            CLOSED_LOOP,
            INVERSE_KINEMATICS
        };

        bool underMode, lockMode, limsOverride, direction;
        ControlMode currentMode, prevMode;
        int buttonInput;

    public:
        IK() {
            WristPos = {0, 0, 0};
            GripperPos = {0, 0, 0};
            wrist = {0,0,0};

            J1 = {0, 0, J1_FWD_LIM, J1_REV_LIM, 1};
            J2 = {60, 0, J2_FWD_LIM, J2_REV_LIM, 2};
            J3 = {-100, 0, J3_POS_LIM, J3_NEG_LIM, 3};
            J4 = {90, 0, J4_FWD_LIM, J4_REV_LIM, 4};
            Pitch = {30, 0, PITCH_FWD_LIM, PITCH_REV_LIM, 5};
            Valkyrie = {0, 0, 1000, -1000, 6};
 
            underMode = lockMode = 0;
            limsOverride = false;
            direction = false;
            buttonInput = 0;
            currentMode = OPEN_LOOP;
            prevMode = OPEN_LOOP;
            J2.model = LoadModel("J2Model.obj");
            J3.model = LoadModel("J3Model.obj");
            J4.model = LoadModel("J4Model.obj");
            Pitch.model = LoadModel("PitchModel.obj");
            Valkyrie.model = LoadModel("ValkModel.obj");
            Solenoid.model = LoadModel("SolModel.obj");
        }

        void Draw();
        void Unload();
        void TransformArm();
        void Keyboard();
        void CalculateIK();
        bool atFwdLim(joint J);
        bool atRevLim(joint J);
        bool isOutsideTargetRange(joint J);
        void LimitJoint(joint &J);
        void UpdateJoint(joint &J);
        void Update();
        void CalculateApparents();
        void UpdateRayLibMatrix(joint &J);
};

#endif