#include <Romi32U4.h>
#include "IR_sensor.h"


void IRsensor::Init(void)
{
    pinMode(pin_IR, INPUT);
}

float IRsensor::PrintData(void)
{
    Serial.println(ReadData());
}

float IRsensor::ReadData(void)
{
  //assignment 1.1
  //read out and calibrate your IR sensor, to convert readouts to distance in [cm]
    int infRedRead = analogRead(pin_IR); 
    //float infRedMath = 8.314*pow(10,-9)*pow(infRedRead,4) - 9.736*pow(10,-6)*pow(infRedRead,2) - 0.824*infRedRead + 71.1687;
    float infRedMath = 99.4676 * pow(0.990116, infRedRead) + 11.6965;
    //Serial.println(infRedRead);
    /*Serial.print(infRedRead);
    Serial.print('\t');
    Serial.print(infRedMath);
    Serial.print('\t');
    */
    
    delay(50);
  
  return infRedMath;
} 
