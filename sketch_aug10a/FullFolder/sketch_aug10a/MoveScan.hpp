#pragma once

#include <Arduino.h>

float lastDistance = 0;
float goalDistance = 5;

void TravelHalfTo(float distance)
{
  if(distance == lastDistance/2)
    lastDistance = distance;

 if(distance < goalDistance)
 {
  //Move Forward
 }
 else if(distance > goalDistance)
 {
  //Move Backward
 }
 if(distance == goalDistance)
 {
  //Catch
  Catch();
 }
}

void Catch()
{
  
}
