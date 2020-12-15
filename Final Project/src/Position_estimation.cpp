#include  "Position_estimation.h"
#include "Encoders.h"

Encoder RomiEncoders;
float x = 0;
float y = 0;
float theta = 0;
unsigned long time_prev = millis();
unsigned long time_now = 0;

void Position::Init(void)
{
    time_prev = millis();
    x = 0;
    y = 0;
    theta = 0;
}

void Position::Stop(void)
{
    time_prev = millis();
    x = 0; 
    y = 0;
    theta = 0;
}

Position::pose_data Position::ReadPose(void)
{
    return {x,y,theta};
}

void Position::PrintPose(void)
{
    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.print(theta);
    Serial.print('\t');
    Serial.print(x_theoretical);
    Serial.print('\t');
    Serial.print(y_theoretical);
    Serial.print('\t');
    Serial.println(theta_theoretical);
    
}

void Position::UpdatePose(float target_speed_left, float target_speed_right)
{
    time_now = millis();
    if(time_now - time_prev >= 50) //update every 50ms for practical reasons
    {
        //assignment
        //measured
        float ICC = (l/2)*(RomiEncoders.ReadVelocityRight()+RomiEncoders.ReadVelocityLeft())/(RomiEncoders.ReadVelocityRight()-RomiEncoders.ReadVelocityLeft());
        float w = (RomiEncoders.ReadVelocityRight()-RomiEncoders.ReadVelocityLeft())/l;
        float v = w*ICC;
        float v_straight = (RomiEncoders.ReadVelocityLeft()+RomiEncoders.ReadVelocityRight())/2;
        //theoretical
        
        float ICC_theoretical = (l/2)*(target_speed_right+target_speed_left)/(target_speed_right-target_speed_left);
        float w_theoretical = (target_speed_right-target_speed_left)/l;
        float v_theoretical = w_theoretical*ICC_theoretical;
        float v_straight_theoretical = target_speed_left;
        
/*Serial.print(ICC);
Serial.print('\t');
Serial.println(ICC_theoretical);
*/
        //straight paths
        //measured
        if (RomiEncoders.ReadVelocityRight() == RomiEncoders.ReadVelocityLeft()){
            state = 1;
            if (prev_state == 1){
                x_straight = x_straight;
                y_straight = y_straight;
                x_theoretical_straight = x_theoretical_straight;
                y_theoretical_straight = y_theoretical_straight;
                theta_straight = theta_straight;
                theta_theoretical_straight = theta_theoretical_straight;
                v_straight = v_straight;
                v_straight_theoretical = v_straight_theoretical;
            }
            else {
                float x_straight = x_curved;
                float y_straight = y_curved;
                float x_theoretical_straight = x_theoretical_curved;
                float y_theoretical_straight = y_theoretical_curved;
                float theta_straight = theta_curved;
                float theta_theoretical_straight = theta_theoretical_curved;
                float v_straight = v;
                float v_theoretical_straight = v_theoretical;
            }
            float nx = x_straight + 0.05*v_straight*cos(theta);
            float ny = y_straight + 0.05*v_straight*sin(theta);
            //theta is constant

            x_straight = nx;
            y_straight = ny;

            x = x_straight;
            y = y_straight;           
            //theoretical
            
            float nx_theoretical = x_theoretical_straight + v_straight_theoretical*cos(theta_theoretical)*0.05;
            float ny_theoretical = y_theoretical_straight + v_straight_theoretical*sin(theta_theoretical)*0.05;

            x_theoretical_straight = nx_theoretical;
            y_theoretical_straight = ny_theoretical;

            x_theoretical = x_theoretical_straight;
            y_theoretical = y_theoretical_straight;
            //theta is constant
            prev_state = 1;
            

        }
        //curved paths
       else if (RomiEncoders.ReadVelocityRight() != RomiEncoders.ReadVelocityLeft()){
            state = 2;
            if (prev_state == 1){
                float x_curved = x_straight;
                float y_curved = y_straight;
                float x_theoretical_curved = x_theoretical_straight;
                float y_theoretical_curved = y_theoretical_straight;
                float theta_curved = theta_straight;
                float theta_theoretical_curved = theta_theoretical_straight;
                ICC = ICC;
                w = w;
                ICC_theoretical = ICC_theoretical;
                w_theoretical = w_theoretical;
                v = v;
                v_theoretical = v_theoretical;
            }
            else {
                x_curved = x_curved;
                y_curved = y_curved;
                x_theoretical_curved = x_theoretical_curved;
                y_theoretical_curved = y_theoretical_curved;
                theta_curved = theta_curved;
                theta_theoretical_curved = theta_theoretical_curved;
                float v = v_straight;
                float v_theoretical = v_straight_theoretical;
                ICC = ICC;
                w = w;
                ICC_theoretical = ICC_theoretical;
                w_theoretical = w_theoretical;
            }
            
            //measured
            float nx = x_curved -ICC*sin(theta) + ICC*sin(theta + 0.05*w);
            float ny = y_curved -ICC*cos(theta) + ICC*cos(theta+ 0.05*w);
            float ntheta = theta_curved + 0.05*w;

            x_curved = nx;
            y_curved = ny;
            theta_curved = ntheta; 

            x = x_curved;
            y = y_curved;
            theta = theta_curved;
            //theoretical
            
            float nx_theoretical = x_theoretical_curved -ICC_theoretical*sin(theta_theoretical) + ICC_theoretical*sin(theta_theoretical + 0.05*w_theoretical);
            float ny_theoretical = y_theoretical_curved -ICC_theoretical*cos(theta_theoretical) + ICC_theoretical*cos(theta_theoretical + 0.05*w_theoretical);
            float ntheta_theoretical = theta_theoretical_curved + 0.05*w_theoretical;
        
            x_theoretical_curved = nx_theoretical;
            y_theoretical_curved = ny_theoretical;
            theta_theoretical_curved = ntheta_theoretical;

            x_theoretical = x_theoretical_curved;
            y_theoretical = y_theoretical_curved;
            theta_theoretical = theta_theoretical_curved;
            prev_state = 2;
            


        }
           
    }

}

