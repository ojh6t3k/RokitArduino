/*
  Name: Biped Gait Tunner (Dynamixel)
  B/D: Ardu-X 200
  Author: Jaehong (jhoh@robolink.co.kr)
  Copyright (C) 2014 RoboLink.  All rights reserved.
*/

#include <Ansiterm.h>
#include <BipedGait.h>
#include <Dynamixel.h>

Ansiterm ansi;
boolean processStart = false;
float legLengthRatio = 0.93;
float hipAngle = -16;
float cycleTime = 0.5;
float dspRatio = 0.28;
float hipSwingRatio = 0.4;
float footSwingRatio = 0.3;
float pelvisSwingRatio = 0.15;
float forwardStep = 0;
float rightStep = 0;
float turnStep = 0;
float balanceRoll = 0;
float balancePitch = 0;
boolean updateStatus = false;
int minRow = 2;
int maxRow = 14;
int currentRow;
int currentColumn = 26;

void callbackMotorControl()
{
  Dynamixel.beginPose(POSE_POS);
  Dynamixel.addPose(7, (word)BipedGait.getMotor(R_HIP_YAW));
  Dynamixel.addPose(8, (word)BipedGait.getMotor(L_HIP_YAW));  
  Dynamixel.addPose(9, (word)BipedGait.getMotor(R_HIP_PITCH));
  Dynamixel.addPose(10, (word)BipedGait.getMotor(L_HIP_PITCH));
  Dynamixel.addPose(11, (word)BipedGait.getMotor(R_HIP_ROLL));
  Dynamixel.addPose(12, (word)BipedGait.getMotor(L_HIP_ROLL));
  Dynamixel.addPose(13, (word)BipedGait.getMotor(R_KNEE_PITCH));
  Dynamixel.addPose(14, (word)BipedGait.getMotor(L_KNEE_PITCH));
  Dynamixel.addPose(15, (word)BipedGait.getMotor(R_ANKLE_PITCH));
  Dynamixel.addPose(16, (word)BipedGait.getMotor(L_ANKLE_PITCH));
  Dynamixel.addPose(17, (word)BipedGait.getMotor(R_ANKLE_ROLL));
  Dynamixel.addPose(18, (word)BipedGait.getMotor(L_ANKLE_ROLL));
  Dynamixel.endPose();
}

void setup()
{
  BipedGait.attach(callbackMotorControl);
  BipedGait.setKinematic(7.7, 9.3, 7.8);
  // angle2ValueRatio: 1024 / 300 = 3.413
  BipedGait.setMotor(R_HIP_YAW, 358, 3.413, -1);
  BipedGait.setMotor(R_HIP_ROLL, 512, 3.413, 1);
  BipedGait.setMotor(R_HIP_PITCH, 512, 3.413, 1);
  BipedGait.setMotor(R_KNEE_PITCH, 512, 3.413, 1);
  BipedGait.setMotor(R_ANKLE_PITCH, 512, 3.413, -1);
  BipedGait.setMotor(R_ANKLE_ROLL, 512, 3.413, 1);
  BipedGait.setMotor(L_HIP_YAW, 666, 3.413, -1);
  BipedGait.setMotor(L_HIP_ROLL, 512, 3.413, 1);
  BipedGait.setMotor(L_HIP_PITCH, 512, 3.413, 1);
  BipedGait.setMotor(L_KNEE_PITCH, 512, 3.413, 1);
  BipedGait.setMotor(L_ANKLE_PITCH, 512, 3.413, -1);
  BipedGait.setMotor(L_ANKLE_ROLL, 512, 3.413, 1);
  UpdateParameters();
  
  Dynamixel.attachPins(2, HIGH, LOW);
  Dynamixel.attachSerial(&Serial1);
  Dynamixel.begin(1000000);
  
  Serial.begin(57600);
  // Draw screen
  ansi.eraseScreen();
  ansi.home();  
  ansi.setBackgroundColor(BLUE);
  ansi.setForegroundColor(GREEN);
  Serial.println("-----------------Biped Gait Tunner------------------");
  ansi.setBackgroundColor(BLACK);
  ansi.setForegroundColor(WHITE);
  Serial.println("Leg length ratio:");
  Serial.println("Hip Angle(deg):");
  Serial.println("Cycle time(sec):");
  Serial.println("DSP ratio:");
  Serial.println("Hip swing ratio:");
  Serial.println("Pelvis swing ratio:");
  Serial.println("Foot swing ratio:");
  Serial.println("Start/Stop:");
  Serial.println("Forward(cm):");
  Serial.println("Right(cm):");
  Serial.println("Turn(deg):");
  Serial.println("Balance Roll(deg):");
  Serial.println("Balance Pitch(deg):");
  ansi.setForegroundColor(GREEN);
  updateStatus = true;
  currentRow = minRow;
}

void loop()
{  
  BipedGait.process();  
  
  // input
  while(Serial.available())
  {
    int keyInput = Serial.read();
    if(keyInput == 27)
    {
      while(Serial.available() == 0);
      int keyInput2 = Serial.read();
      if(keyInput2 == 91)
      {
        while(Serial.available() == 0);
        int keyInput3 = Serial.read();
        if(keyInput3 == 65) // up arrow
        {
          currentRow--;
          if(currentRow < minRow)
            currentRow = minRow;
        }
        else if(keyInput3 == 66) // down arrow
        {
          currentRow++;
          if(currentRow > maxRow)
            currentRow = maxRow;
        }
      }
    }
    else if(keyInput == '[')
    {
      int row = minRow;
      if(currentRow == row++)
        legLengthRatio -= 0.01;
      else if(currentRow == row++)
        hipAngle -= 0.1;
      else if(currentRow == row++)
        cycleTime -= 0.01;
      else if(currentRow == row++)
        dspRatio -= 0.01;
      else if(currentRow == row++)
        hipSwingRatio -= 0.01;
      else if(currentRow == row++)
        pelvisSwingRatio -= 0.01;
      else if(currentRow == row++)
        footSwingRatio -= 0.01;
      else if(currentRow == row++)
      {
        processStart = false;
        BipedGait.stop();
      }
      else if(currentRow == row++)
        forwardStep -= 0.1;
      else if(currentRow == row++)
        rightStep -= 0.1;
      else if(currentRow == row++)
        turnStep -= 1;
      else if(currentRow == row++)
        balanceRoll -= 1;
      else if(currentRow == row++)
        balancePitch -= 1;
    }
    else if(keyInput == ']')
    {
      int row = minRow;
      if(currentRow == row++)
        legLengthRatio += 0.01;
      else if(currentRow == row++)
        hipAngle += 0.1;
      else if(currentRow == row++)
        cycleTime += 0.01;
      else if(currentRow == row++)
        dspRatio += 0.01;
      else if(currentRow == row++)
        hipSwingRatio += 0.01;
      else if(currentRow == row++)
        pelvisSwingRatio += 0.01;
      else if(currentRow == row++)
        footSwingRatio += 0.01;
      else if(currentRow == row++)
      {
        processStart = true;
        BipedGait.start();
      }
      else if(currentRow == row++)
        forwardStep += 0.1;
      else if(currentRow == row++)
        rightStep += 0.1;
      else if(currentRow == row++)
        turnStep += 1;
      else if(currentRow == row++)
        balanceRoll += 1;
      else if(currentRow == row++)
        balancePitch += 1;
    }
    updateStatus = true;
  }
  
  // Draw status
  if(updateStatus == true)
  {
    UpdateParameters();
    updateStatus = false;
    int row = minRow;
    DrawStatusFloat(row++, legLengthRatio);
    DrawStatusFloat(row++, hipAngle);
    DrawStatusFloat(row++, cycleTime);
    DrawStatusFloat(row++, dspRatio);
    DrawStatusFloat(row++, hipSwingRatio);
    DrawStatusFloat(row++, pelvisSwingRatio);
    DrawStatusFloat(row++, footSwingRatio);
    DrawStatusBoolean(row++, processStart);
    DrawStatusFloat(row++, forwardStep);
    DrawStatusFloat(row++, rightStep);
    DrawStatusFloat(row++, turnStep);
    DrawStatusFloat(row++, balanceRoll);
    DrawStatusFloat(row++, balancePitch);
    ansi.xy(currentColumn - 1, currentRow); // cursor    
  }
}

void DrawStatusFloat(int row, float value)
{
  ansi.xy(currentColumn, row);
  Serial.print("                "); //erase
  ansi.xy(currentColumn, row);
  Serial.print(value, 3);
}

void DrawStatusBoolean(int row, boolean value)
{
  ansi.xy(currentColumn, row);
  ansi.xy(currentColumn, row);
  if(value == true)
      Serial.print("ON ");
    else
      Serial.print("OFF");    
}

void UpdateParameters()
{
  legLengthRatio = clamp(legLengthRatio, 0.6, 1);
  hipAngle = clamp(hipAngle, -30, 30);
  cycleTime = clamp(cycleTime, 0.1, 5);
  dspRatio = clamp(dspRatio, 0, 1);
  hipSwingRatio = clamp(hipSwingRatio, 0.1, 1);
  footSwingRatio = clamp(footSwingRatio, 0, 1);
  forwardStep = clamp(forwardStep, -5, 5);
  rightStep = clamp(rightStep, -5, 5);
  turnStep = clamp(turnStep, -30, 30);
  balanceRoll = clamp(balanceRoll, -30, 30);
  balancePitch = clamp(balancePitch, -30, 30);
  
  BipedGait.setPose(legLengthRatio, hipAngle);
  BipedGait.tuneSwing(cycleTime, dspRatio, hipSwingRatio, footSwingRatio, pelvisSwingRatio);
  BipedGait.control(forwardStep, rightStep, turnStep);
  BipedGait.balance(balanceRoll, balancePitch);
}

float clamp(float value, float minValue, float maxValue)
{
  if(value < minValue)
    value = minValue;
  else if(value > maxValue)
    value = maxValue;
  
  return value;
}
