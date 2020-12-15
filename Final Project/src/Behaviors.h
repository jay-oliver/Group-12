#ifndef BEHAVIORS
#define BEHAVIORS

#include <Romi32U4.h>

class Behaviors{
    private:
        int threshold = -1800;
        int threshold_pick_up = 23000;
        int threshold_off_ramp = 100;
        int threshold_on_ramp = 200;
        int data[3] = {0};
        enum ROBOT_STATE {STEP1, STEP2A, STEP2B, STEP3, STEP4, STEP5, STEP6};
        ROBOT_STATE robot_state = STEP1; //initial state: IDLE
        bool go;
        uint32_t now = 0;
        bool timerUp = false;
         
    public:
        void Init(void);
        void Stop(void);
        void Run(void);
        boolean DetectCollision(void);
        boolean DetectBeingPickedUp(void);
        boolean DetectOffRamp(void);
        boolean DetectOnRamp(void);
        void Problem1(void);
        void Move(int, int); //speed, time
        void FollowWall(void);

};

#endif