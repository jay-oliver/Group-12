#include <Arduino.h>
#include "Behaviors.h"



Behaviors positionEstimation;
int now;
bool timerUp;

void setup() {
  positionEstimation.Init();
  now = millis();
}

void loop() {
positionEstimation.Run();
//positionEstimation.Move(100, 6);
//positionEstimation.FollowWall();

//positionEstimation.Turn(90, 0);
/*if (millis()- now >= 1000){
        timerUp = true;  
        if(timerUp){
            if(positionEstimation.DetectCollision()){
                positionEstimation.Stop();
                //Serial.println("Step 2 complete");
                //robot_state = STEP3;
            }
            else{
                positionEstimation.Go(100, 100);
                //Serial.println();
            }
      }
    }*/
}
  