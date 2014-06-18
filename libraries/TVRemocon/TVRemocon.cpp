/*
  TVRemocon.cpp - TVRemocon library
  Copyright (C) 2014 RoboLink.  All rights reserved.
*/

#include "Arduino.h"
#include "TVRemocon.h"

unsigned long _recvData = 0;
unsigned char _key = 0;
unsigned int _CID = 0;
unsigned long _lastTime = 0;
unsigned long _limitTime = 0;

IRrecv _irrecv(RECV_PIN);
decode_results _results;

TVRemoconClass::TVRemoconClass()
{
	
}

void TVRemoconClass::begin()
{
	_irrecv.enableIRIn(); 
}

unsigned char TVRemoconClass::receive()
{
   unsigned long _pastTime = 0;
   unsigned long now = millis();
   unsigned char temp = 0;
   
   _pastTime = now - _lastTime;  
           
   if (_irrecv.decode(&_results))
   {
     _limitTime = 0;
     _recvData = _results.value;
     _CID = (_recvData & 0xffff0000) >> 16;
     temp = (_recvData & 0x0000ff00) >> 8;
     if(temp != 0) _key = temp;
    
     _irrecv.resume(); 
   }
   else
   {
     _limitTime = _limitTime + _pastTime;
     if (_limitTime >= LIMIT_TIME) 
     {
       _key = BUTTON_UP;
     }     
   }
   
  _lastTime = now;

  return _key;
}

unsigned int TVRemoconClass::customCode()
{
	return _CID;
}


 TVRemoconClass TVRemocon;
