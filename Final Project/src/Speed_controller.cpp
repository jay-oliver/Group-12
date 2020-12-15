#include <Romi32U4.h>
#include "Encoders.h"
#include  "Speed_controller.h"
#include "Position_estimation.h"
#include "Behaviors.h"

Romi32U4Motors motors;
Encoder MagneticEncoder; 
Position odometry;
Behaviors behaviors;

void SpeedController::Init(void)
{
    MagneticEncoder.Init();
    odometry.Init();
}

void SpeedController::Run(float target_velocity_left, float target_velocity_right)
{
    if(MagneticEncoder.UpdateEncoderCounts()){
        float e_left = target_velocity_left - MagneticEncoder.ReadVelocityLeft();
        float e_right = target_velocity_right - MagneticEncoder.ReadVelocityRight();

        E_left = E_left + e_left;
        E_right = E_right + e_right;

        float u_left = Kp*e_left + Ki*E_left;
        float u_right = Kp*e_right + Ki*E_right;

        motors.setEfforts(u_left,u_right);
        odometry.UpdatePose(target_velocity_left,target_velocity_right); //this is where your newly programmed function is/will be called
        odometry.PrintPose();
        
    }
}

void SpeedController::Process(float target_velocity_left, float target_velocity_right)
{
    if(MagneticEncoder.UpdateEncoderCounts()){
        float e_left_process = target_velocity_left - MagneticEncoder.ReadVelocityLeft();
        float e_right_process = target_velocity_right - MagneticEncoder.ReadVelocityRight();

        E_left_process += e_left_process;
        E_right_process += e_right_process;

        float u_left = Kp_process*e_left_process + Ki_process*E_left_process;
        float u_right = Kp_process*e_right_process + Ki_process*E_right_process;

        motors.setEfforts(u_left,u_right);
       /* Serial.print(MagneticEncoder.ReadVelocityLeft());
        Serial.print('\t');
        Serial.println(MagneticEncoder.ReadVelocityRight());
        */
    }
} 

boolean SpeedController::Turn(int degree, int direction)
{
    motors.setEfforts(0, 0);
    int turns = counts*degree; //assignment 1: convert degree into counts
    int count_turn = MagneticEncoder.ReadEncoderCountLeft();

    while(abs(abs(count_turn) - abs(MagneticEncoder.ReadEncoderCountLeft())) <= turns)
    {
        if(!direction) {
            Run(50,-50);
        }
        else {
            Run(-50,50);
        }
        Serial.println("turning");
    }
    motors.setEfforts(0, 0);
    Serial.println("done turning");
    return 1;
}

boolean SpeedController::Straight(int target_velocity, int time) //in mm/s and s
{
    //motors.setEfforts(0, 0);
    unsigned long now = millis();

    while ((millis() - now) <= time*1000){
        Run(target_velocity,target_velocity);
        Serial.println("driving straight");
    }
    motors.setEfforts(0, 0);
    return 1;
}

boolean SpeedController::Curved(int target_velocity_left, int target_velocity_right, int time) //in mm/s and s
{
    motors.setEfforts(0, 0);
    
    unsigned long now = millis();

    while ((unsigned long)(millis() - now) <= time*1000){
        Run(target_velocity_left,target_velocity_right);
    }
    motors.setEfforts(0, 0);
    return 1;
}
 boolean SpeedController::Ramp(int target_velocity){
     if(behaviors.DetectOffRamp()){
        Stop();
     }
     else{
         Run(target_velocity, target_velocity);
         Serial.println("driving on ramp");
     }
 }
void SpeedController::Stop()
{
    motors.setEfforts(0,0);
    odometry.Stop();
}