/*
  TVRemocon.h - TVRemocon library
  Copyright (C) 2014 RoboLink.  All rights reserved.
*/
#ifndef TVRemocon_h
#define TVRemocon_h

#include "IRremote.h"

// For hunoi16 Setup

#define  RECV_PIN    11
#define  BUTTON_UP   0xFF
#define  LIMIT_TIME  200

class TVRemoconClass
{
  public:
	TVRemoconClass(void);
	void begin(void);
	unsigned char receive(void);
	unsigned int customCode(void);

  private:
	
    
};

extern TVRemoconClass TVRemocon;

#endif 