/*
  Name: dxl_write
  B/D: Ardu-X 200
  Author: Jaehong (jhoh@robolink.co.kr)
  Copyright (C) 2014 RoboLink.  All rights reserved.
*/

#include <Dynamixel.h>

#define DIR_PIN  2
#define DIR_TX  HIGH
#define DIR_RX  LOW

#define DXL_ID  1
#define DXL_BAUD 1000000
#define MIN_POS  0
#define MAX_POS  1023

boolean toggle = false;

void setup()
{
  Dynamixel.attachSerial(&Serial1);
  Dynamixel.attachPins(DIR_PIN, DIR_TX, DIR_RX);
  Dynamixel.begin(DXL_BAUD);
}

void loop()
{
  if(toggle == false)
  {
    Dynamixel.joint(DXL_ID, MIN_POS);
    toggle = true;
  }
  else
  {
    Dynamixel.joint(DXL_ID, MAX_POS);
    toggle = false;
  }
  
  delay(1000);
}
