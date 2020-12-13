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
    Serial.println(theta);
}

void Position::UpdatePose(float target_speed_left, float target_speed_right)
{
    time_now = millis();
    if(time_now - time_prev >= 50) //update every 50ms for practical reasons
    {
        //assignment
        //measured
        ICC = (l/2)*(RomiEncoders.ReadVelocityRight()+RomiEncoders.ReadVelocityLeft())/(RomiEncoders.ReadVelocityRight()-RomiEncoders.ReadVelocityLeft());
        w = (RomiEncoders.ReadVelocityRight()-RomiEncoders.ReadVelocityLeft())/l;
        v = w*ICC;

        //theoretical
        /*
        ICC_theoretical = (l/2)*(target_speed_right+target_speed_left)/(target_speed_right-target_speed_left);
        w_theoretical = (target_speed_right-target_speed_left)/l;
        v_theoretical = w_theoretical*ICC_theoretical;
        */

        //straight paths
        //measured
        if (RomiEncoders.ReadVelocityRight() == RomiEncoders.ReadVelocityLeft()){
            int nx = x + 50*v*cos(theta);
            int ny = y + 50*v*sin(theta);
            //theta is constant

            x = nx;
            y = ny;           
            //theoretical
            /*
            x_theoretical = x_theoretical + v_theoretical*cos(theta_theoretical)*50;
            y_theoretical = y_theoretical + v_theoretical*sin(theta_theoretical)*50;
            */
            //theta is constant
            /*
            Serial.println(x_measured);
            Serial.println("\t");
            Serial.println(y_measured);
            Serial.println("\t");
            Serial.println(theta_measured);
            Serial.println("\t");
            Serial.println(x_theoretical);
            Serial.println("\t");
            Serial.println(y_theoretical);
            Serial.println("\t");
            Serial.println(theta_theoretical);
            */
        }
        //curved paths
        else if (RomiEncoders.ReadVelocityRight() != RomiEncoders.ReadVelocityLeft()){
            //measured
            int nx = x -ICC*sin(theta) + ICC*sin(theta + 50*w);
            int ny = y -ICC*cos(theta) + ICC*cos(theta+ 50*w);
            int ntheta = theta + 50*w;

            x = nx;
            y = ny;
            theta = ntheta; 
            //theoretical
            /*
            x_theoretical = x_theoretical -ICC_theoretical*sin(theta_theoretical) + ICC_theoretical*sin(theta_theoretical + 50*w_theoretical);
            y_theoretical = y_theoretical -ICC_theoretical*cos(theta_theoretical) + ICC_theoretical*cos(theta_theoretical + 50*w_theoretical);
            theta_theoretical = theta_theoretical + 50*w_theoretical;
        */

        }
            /*
            Serial.println(x);
            Serial.println("\t");
            Serial.println(y);
            Serial.println("\t");
            Serial.println(theta);
            Serial.println("\t");
            */
            
    }
}
