#include <Arduino.h>
#include "Behaviors.h"


Behaviors positionEstimation;


void setup() {
  positionEstimation.Init();
}

void loop() {
//positionEstimation.Run();
//positionEstimation.Move(30, 30);
positionEstimation.FollowWall();
}
  