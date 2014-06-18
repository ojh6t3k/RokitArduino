/*
  SineGait.h - Sinusoidal motion control library
  Copyright (C) 2014 Robolink.  All rights reserved.
*/


//******************************************************************************
//* Includes
//******************************************************************************
#include "SineGait.h"
#include <math.h>


//******************************************************************************
//* Constructors
//******************************************************************************

SineGait::SineGait(int jointNum)
{
	if(jointNum > 0 && jointNum <= MAX_SINEGAIT_JOINT)
		_jointNum = jointNum;
	else
		_jointNum = MAX_SINEGAIT_JOINT;
	_run = false;
	for(int i=0; i<_jointNum; i++)
		_angle[i] = 0;
}

//******************************************************************************
//* Public Methods
//******************************************************************************

void SineGait::setPeriod(float time)
{
	if(time > 0)
		_periodTime = time;
}

void SineGait::setJoint(int index, float amplitude, float balance, float phase, float speed)
{
	if(index >= 0 && index < _jointNum)
	{
		_amplitude[index] = amplitude;
		_balance[index] = balance;
		_phase[index] = phase;
		_speed[index] = speed;
	}
}

float SineGait::getAngle(int index)
{
	if(index >= 0 && index < _jointNum)
	{
		return _angle[index];
	}
	else
		return 0;
}

int SineGait::getValue(int index, int center, float scale, int dir)
{
	if(index >= 0 && index < _jointNum)
	{
		int offset = (int)(_angle[index] * scale);
		return center + dir * offset;
	}
	else
		return 0;	
}

void SineGait::start()
{
	if(_run == true)
		return;

	_run = true;
	_time = 0;
	_stop = false;
	_preMillis = millis();
}

void SineGait::stop()
{
	_stop = true;
}

void SineGait::process()
{
	if(_run == false)
		return;

	 // update delta time
	unsigned long curMillis = millis();
	_time += (float)(curMillis - _preMillis) / 1000;
	_preMillis = curMillis;

	if(_time > _periodTime)
	{
		_time = _time - _periodTime;
		if(_stop == true)
		{
			_run = false;
			_time = 0;
		}
	}

	float timeRatio = _time / _periodTime;
	for(int i=0; i<_jointNum; i++)
	{
		float jointTimeRatio = timeRatio + _phase[i];
		if(jointTimeRatio < 0)
			jointTimeRatio = 1 - jointTimeRatio;
		else if(jointTimeRatio > 1)
			jointTimeRatio = jointTimeRatio - 1;
		_angle[i] = _amplitude[i] * sin((_speed[i] * jointTimeRatio) * 2 * M_PI);
		if(jointTimeRatio < 0.5 && _balance[i] > 0)
			_angle[i] *= 1 - _balance[i];
		else if(jointTimeRatio > 0.5 && _balance[i] < 0)
			_angle[i] *= 1 + _balance[i];
	}
}

//******************************************************************************
//* Private Methods
//******************************************************************************
