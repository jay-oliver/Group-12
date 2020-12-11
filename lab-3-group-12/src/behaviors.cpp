#include <Romi32U4.h>
#include "Behaviors.h"
#include "Median_filter.h"
#include "IMU.h"
#include "Speed_controller.h"
#include "Encoders.h"

//sensors
IMU_sensor LSM6;
Romi32U4ButtonA buttonA;
Encoder encoder;
//median filter
MedianFilter med_x;
MedianFilter med_y;
MedianFilter med_z;

//motor-speed controller
SpeedController PIcontroller;

//state machine
enum ROBOT_STATE {IDLE, DRIVE, REVERSE, TURN};
ROBOT_STATE robot_state = IDLE;

void Behaviors::Init(void)
{
    LSM6.Init();
    med_x.Init();
    med_y.Init();
    med_z.Init();
    PIcontroller.Init();
}

boolean Behaviors::DetectCollision(void)
{
    auto data_acc = LSM6.ReadAcceleration();
    data[0] = med_x.Filter(data_acc.X)*0.061;
    data[1] = med_y.Filter(data_acc.Y)*0.061;
    data[2] = med_z.Filter(data_acc.Z)*0.061;
    if((abs(data[0]) > threshold) || (abs(data[1]) > threshold)){
         return true;
    }
    else 
        return false;
}

boolean Behaviors::DetectBeingPickedUp(void)
{
    auto data_acc = LSM6.ReadAcceleration();
    if( data_acc.Z >= threshold_pick_up){
        return true;
    }
    else{
        return false;
    }
}

void Behaviors::Stop(void)
{
    PIcontroller.Stop();
}

void Behaviors::Run(void){

    switch (robot_state)
    {
    case IDLE:
        if(buttonA.getSingleDebouncedRelease())
        { //transition condition
            robot_state = DRIVE; 
            PIcontroller.Stop(); //action
            Serial.println("drive");
        } 
        else { //transition condition
            robot_state = IDLE; 
            PIcontroller.Stop(); //action 
            Serial.println("idle");
        }   
        break;
        //assignment 3
    case DRIVE:
       if(buttonA.getSingleDebouncedRelease()){ //transition condition
            robot_state = IDLE;
            PIcontroller.Stop();
            Serial.println(robot_state);
       }
        else if(DetectCollision()){
            robot_state = REVERSE;
            PIcontroller.Stop();
            Serial.println(robot_state);
        }
        else if(DetectBeingPickedUp()){
            robot_state = IDLE;
            PIcontroller.Stop();
            Serial.println(robot_state);
        }
        else {
            robot_state = DRIVE;
            PIcontroller.Run(50,50);
            Serial.println(robot_state);
        }
        break;
    case REVERSE:
        if(buttonA.getSingleDebouncedRelease()){
            robot_state = IDLE;
            PIcontroller.Stop();
            Serial.println(robot_state);
        }
        else if(!PIcontroller.Reverse(100,15)){
            robot_state = REVERSE;
            PIcontroller.Reverse(100, 15);
            Serial.println(robot_state);
        }
        else{
            robot_state = TURN;
            PIcontroller.Stop();
            Serial.println(robot_state);
        } 
        break;
    case TURN:
        if(buttonA.getSingleDebouncedRelease()){
            robot_state = IDLE;
            PIcontroller.Stop();
            Serial.println(robot_state);
        }
        else if(encoder.UpdateEncoderCounts()){
            robot_state = DRIVE;
            PIcontroller.Stop();
            Serial.println(robot_state);
        }
        else{
            robot_state = TURN;
            PIcontroller.Turn(90, true);
            Serial.println(robot_state);
        }
        break;
    }
    
}