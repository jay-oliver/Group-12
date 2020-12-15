#include <Romi32U4.h>
#include "Behaviors.h"
#include "Speed_controller.h"
#include "Position_estimation.h"
#include "Wall_following_controller.h"
#include "Median_filter.h"
#include "IMU.h"
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
WallFollowingController wallfollow;

//state machine
enum ROBOT_STATE {STEP1, STEP2A, STEP2B, STEP3, STEP4, STEP5, STEP6};
ROBOT_STATE robot_state = STEP1;




void Behaviors::Init(void)
{
    LSM6.Init();
    med_x.Init();
    med_y.Init();
    med_z.Init();
    PIcontroller.Init();
    wallfollow.Init();
}

boolean Behaviors::DetectCollision(void)
{
    auto data_acc = LSM6.ReadAcceleration();
    data[0] = med_x.Filter(data_acc.X)/**0.061*/;
    data[1] = med_y.Filter(data_acc.Y)/**0.061*/;
    data[2] = med_z.Filter(data_acc.Z)/**0.061*/;
    if((data[0] < threshold) || (data[1] < threshold)){
         return true;
    }
    else 
        return false;
}

boolean Behaviors::DetectBeingPickedUp(void)
{
    auto data_acc = LSM6.ReadAcceleration();
    if( data_acc.Z >= threshold_pick_up){
        Serial.println("I was picked up");
        return true;
    }
    else{
        return false;
    }
}

boolean Behaviors::DetectOffRamp(void){
     auto data_acc = LSM6.ReadAcceleration();
    if( data_acc.Z >= threshold_off_ramp){
        Serial.println("off ramp");
        return true;
    }
    else{
        return false;
    }
}

boolean Behaviors::DetectOnRamp(void){
    auto data_acc = LSM6.ReadAcceleration();
    if( data_acc.Z >= threshold_on_ramp){
        Serial.println("on ramp");
        return true;
    }
    else{
        return false;
    }
}
void Behaviors::Stop(void)
{
    PIcontroller.Stop();
    Serial.println("stopped");
}

void Behaviors::Move(int speed, int time){
    if (buttonA.getSingleDebouncedRelease()){
        PIcontroller.Straight(speed, time);
    }
}

void Behaviors::FollowWall(void){
    float speed = wallfollow.Process(1);
    PIcontroller.Process(50+speed, 50-speed);
}

void Behaviors::Run(void){

    switch (robot_state)
    {
    case STEP1:
      ////Load Object onto the robot and press button
      if(buttonA.getSingleDebouncedRelease())
      {
        Serial.println("Step One Completed Successfully");
        //Can we please have a happy beep or something that'd be adorable
        robot_state = STEP2A;
      }
      else{
        robot_state = STEP1;
      }
      break;
    case STEP2A:
      ////After one second, move straight until you collide with the wall, then stop
      //millis until one second
     uint32_t now = millis();
      if (millis()- now >= 1000){
        timerUp = true;  
        if(timerUp){
       /* for (int i = 0; i <1000; i++){
            while(millis()-now <= 1){
                PIcontroller.Stop();
            }
            now = millis();
            Serial.println("waiting");
            }
            robot_state = STEP2B;
            Serial.println("Done Waiting");
        break;
        //}
        case STEP2B:*/
            if(DetectCollision()){
                PIcontroller.Stop();
                Serial.println("Step 2 complete");
                robot_state = STEP3;
            }
            else{
                PIcontroller.Run(100, 100);
                Serial.println("no collision yet");
            }
      }
    }
   else{
        timerUp = false;
         Serial.println("waiting for 1 sec");
    }
      break;
    case STEP3:
      ////Remove object from robot, place another object onto the robot, press button
      if(buttonA.getSingleDebouncedRelease()){
        Serial.println("Step Three Completed Successfully");
        //Can we please have a happy beep or something that'd be adorable
        robot_state = STEP4;
      }
      else{
        robot_state = STEP3;
        Serial.println("Please press button");
      }
    break;
    case STEP4:
      ////Rotate 90 degrees, follow wall around the corner
      if(DetectOnRamp()){
        Serial.println("Step 4 completed");
        robot_state = STEP5;
      }
      else{
        if(PIcontroller.Turn(90, 1)){
        wallfollow.Process(20);
        robot_state = STEP4;
      }
    }
      break;
    case STEP5:
      ////Pass the ramp, then continue to follow the wall for another 10 cm
      if(DetectOffRamp()){
        PIcontroller.Stop();
        Serial.println("Step 5 completed");
        robot_state = STEP6;
      }
      else{
        PIcontroller.Ramp(25);
        robot_state = STEP5;
      }
      break;
    case STEP6:
    PIcontroller.Straight(25, 4);
    PIcontroller.Stop();
    Serial.println("Step 6 completed");
      ////Stop, and remove object from robot
      //Happy happy beep maybe possibly?
      break;
}

}