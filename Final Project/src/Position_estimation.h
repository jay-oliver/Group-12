#ifndef POSITION_ESTIMATION
#define POSITION_ESTIMATION

#include <Romi32U4.h>

class Position{
    private:
        float x, y, theta;
        float x_theoretical, y_theoretical, theta_theoretical;
        unsigned long time_prev, time_now;
        const float l = 0.2; //assignment
        //float ICC, w, v; //w is omega I'm sorry
        //float ICC_theoretical, w_theoretical, v_theoretical;
        float v_straight, v_straight_theoretical;
        int prev_state, state;
        float x_theoretical_straight, y_theoretical_straight, x_straight, y_straight;
        float x_theoretical_curved, y_theoretical_curved, x_curved, y_curved;
        float theta_theoretical_straight, theta_theoretical_curved, theta_straight, theta_curved;
        
        
        
    public:
        struct pose_data {
            float X;
            float Y;
            float THETA;
        };
        void Init(void);
        void UpdatePose(float,float);
        pose_data ReadPose(void);
        void PrintPose(void);
        void Stop(void);
};

#endif