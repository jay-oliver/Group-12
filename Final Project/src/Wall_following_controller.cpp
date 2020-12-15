#include <Romi32U4.h>
#include "Encoders.h"
#include "Wall_following_controller.h"
#include "IR_sensor.h"

IRsensor SharpIR;
void WallFollowingController::Init(void)
{
    SharpIR.Init();
}

float WallFollowingController::Process(float target_distance)
{
  //assignment 2: write a PD controller that outputs speed as a function of distance error
  distance = SharpIR.ReadData();
  E_distance = target_distance - distance;
    float de = E_distance - prev_e_distance;
  float speed = constrain(Kp*E_distance + Kd*de, -50, 50) ;
  prev_e_distance = E_distance;
Serial.println("wall following");
  return speed;
} 
