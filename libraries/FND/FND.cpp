/*
  CLCD.cpp - CLCD library
  Copyright (C) 2014 RoboLink.  All rights reserved.
*/

#include "FND.h"
#include "Arduino.h"

// FND
unsigned char FND_Count = 0;	//fnd 출력 자릿수 컨트롤
unsigned char FND_data[4] = {0,0,0,0};

volatile uint8_t *siPort;
volatile uint8_t *rckPort;
volatile uint8_t *sckPort;

void FNDClass::write595(unsigned int data)
{
  unsigned char i=7;
  
  *siPort &= ~digitalPinToBitMask(_si);
  *sckPort &= ~digitalPinToBitMask(_sck);
  *rckPort &= ~digitalPinToBitMask(_rck);
  
  while(i<=7)
  {
    if((data>> i) & 0x01) *siPort |=  digitalPinToBitMask(_si);
    else *siPort &= ~digitalPinToBitMask(_si);
    *sckPort |=  digitalPinToBitMask(_sck);
    *sckPort &= ~digitalPinToBitMask(_sck);
    i--;
  }
    *rckPort |=  digitalPinToBitMask(_rck);
    *rckPort &= ~digitalPinToBitMask(_rck);
}

void FNDClass::begin(int pin1,int pin2,int pin3) //LCD initialize
{
	_si   = pin1;	//data
	_rck  = pin2;	//latch
	_sck  = pin3;	//shift
	
	siPort = portOutputRegister(digitalPinToPort(_si)); 
	rckPort = portOutputRegister(digitalPinToPort(_rck)); 
	sckPort = portOutputRegister(digitalPinToPort(_sck)); 

	pinMode(_si,OUTPUT); 
	pinMode(_rck,OUTPUT); 
	pinMode(_sck,OUTPUT); 
	
	digitalWrite(_si, LOW);
	digitalWrite(_rck, LOW);
	digitalWrite(_sck, LOW);
}

void FNDClass::setNumber(unsigned int value)
{	
	FND_data[3] =(value/1000);
	FND_data[2] =(value%1000)/100; 
	FND_data[1] =(value%100)/10;
	FND_data[0] =(value%10);
}

void FNDClass::dynamicDisplay(void)
{
	write595((0b00010000 << FND_Count ) | FND_data[FND_Count]);
	FND_Count = FND_Count + 1;
	if (FND_Count == 4) FND_Count = 0;
}

void FNDClass::leftShift()
{	
  char temp = 0;
  temp = FND_data[0];
  FND_data[0] = FND_data[1];
  FND_data[1] = FND_data[2];
  FND_data[2] = FND_data[3]; 
  FND_data[3] = temp;
}

void FNDClass::rightShift()
{	
  char temp = 0;
  temp = FND_data[3];
  FND_data[3] = FND_data[2];
  FND_data[2] = FND_data[1]; 
  FND_data[1] = FND_data[0];
  FND_data[0] = temp;
}

FNDClass FND;