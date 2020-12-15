#ifndef BEHAVIORS
#define BEHAVIORS

#include <Romi32U4.h>

class Behaviors{
    private:
        int threshold_high = 24000;
        int threshold_low = 0;
        int threshold_pick_up = 23000;
        int threshold_off_high = 23000;
        int threshold_off_ramp = 5000;
        int threshold_on_ramp_high = 50000;
        int threshold_on_ramp_low = -25000;
        int data[3] = {0};
        enum ROBOT_STATE {STEP1, STEP2, STEP3, STEP4, STEP5, STEP6, STOP};
        ROBOT_STATE robot_state = STEP1; //initial state: IDLE
        bool go;
        uint32_t now = 0;
        bool timerUp = false;
        uint32_t step4Time = 0;
        uint32_t step5Time = 0;
         
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
        void Ramp(int);
        void Turn(int, int);
        void Go (int, int);

};

#endif