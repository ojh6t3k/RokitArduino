/*
  ADCModule.h - UnityRobot Module library
  Copyright (C) 2014 Robolink.  All rights reserved.
*/


//******************************************************************************
//* Includes
//******************************************************************************
#include "UnityRobot.h"
#include "ADCModule.h"


//******************************************************************************
//* Constructors
//******************************************************************************

ADCModule::ADCModule(int id, int pin)
{
	_id = id;
	_pin = pin;
}

//******************************************************************************
//* Public Methods
//******************************************************************************

void ADCModule::flush()
{
	UnityRobot.select(_id);
	UnityRobot.push((word)analogRead(_pin));
	UnityRobot.flush();
}

//******************************************************************************
//* Private Methods
//******************************************************************************
