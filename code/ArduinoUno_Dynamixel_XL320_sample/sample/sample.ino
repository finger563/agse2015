/*
Code to test HelloSpoon behavior.

  1. Initialize the comm.
  2. Turn Dynamixel ID 5 LED ON, Yellow color.
  3. Move Joint #4.
  4. Random positions and LED colors are generated in Loop.

To learn the full list of methods check inside HelloSpoon.h

To learn how to upload this code and custom codes to HelloSpoon
please visit the Wiki included in this repository.

Made by Luis G III for HelloSpoon robot.


*/

#include "HelloSpoon.h"

void setup(){

  hs.begin();
  delay(1000);

  hs.LED(253,2);
  hs.LED(253,2);
  hs.setJointSpeed(253, 1023);
  hs.moveJoint(253, 212);
  delay(1000);

}

void loop(){  

  hs.LED(253,random(1,7));
  delay(500);  
  hs.moveJoint(253, 0);
  delay(500);
  hs.moveJoint(253, 300);
  delay(500);
  hs.moveJoint(253, 600);
  delay(500);
  hs.moveJoint(253, 900);
  delay(500);  
  hs.moveJoint(253, 600);
  delay(500);
  hs.moveJoint(253, 300);
  delay(500);
  hs.moveJoint(253, 0);
  delay(500);
}
