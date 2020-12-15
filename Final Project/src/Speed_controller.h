#ifndef SPEED_CONTROLLER
#define SPEED_CONTROLLER

#include <Romi32U4.h>

class SpeedController{
    private:
        const float Kp = 0.5; //Adapt the parameters until your robot moves at the speed you command it to drive
        const float Ki = 0.1; 
        float E_left = 0; 
        float E_right = 0;
        const float Kp_process = 0.5;
        const float Ki_process = 0.1;
        float E_left_process = 0;
        float E_right_process = 0;
        int counts = 8; //assignment

    public:
        void Init(void);
        void Run(float, float); //speed left, speed right
        boolean Turn(int,int); //degrees, direction of rotation: 0->left, 1->right
        boolean Straight(int, int); //speed, duration
        boolean Curved(int,int,int); //speed left, speed right, duration
        boolean Ramp(int); //speed
        void Process(float, float);
        void Stop(void);
};

#endif