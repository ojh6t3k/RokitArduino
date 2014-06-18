/*
  BipedGait.cpp - BipedGait library
  Copyright (C) 2014 Robolink.  All rights reserved.
*/


//******************************************************************************
//* Includes
//******************************************************************************

#include "BipedGait.h"

extern "C" {
#include <math.h>
}

#define STOP_RIGHT_STEP		0
#define STOP_LEFT_STEP		1
#define START_RIGHT_STEP	2
#define START_LEFT_STEP		3
#define LOOP_RIGHT_STEP		4
#define LOOP_LEFT_STEP		5
#define END_RIGHT_STEP		6
#define END_LEFT_STEP		7


//******************************************************************************
//* Constructors
//******************************************************************************

BipedGaitClass::BipedGaitClass()
{
	_phase = STOP_RIGHT_STEP;
	_preTime = 0;

	setKinematic(1, 1, 1);
	setPose(1, 0);
	tuneSwing(1, 0.8, 1, 0.4, 0);
	balance(0, 0);
	control(0, 0, 0);

	_deg2Rad = M_PI / 180;
	_rad2Deg = 180 / M_PI;
	_m_pi_2 = M_PI * 0.5;
}

//******************************************************************************
//* Public Methods
//******************************************************************************

void BipedGaitClass::start()
{
	if(_phase == STOP_RIGHT_STEP || _phase == STOP_LEFT_STEP)
	{
		if(_phase == STOP_RIGHT_STEP)
			_phase = START_LEFT_STEP;
		else
			_phase = START_RIGHT_STEP;
		_stop = false;
		_time = 0;
		_preTime = 0;
	}
}

void BipedGaitClass::stop()
{
	_stop = true;	
}
	
void BipedGaitClass::attach(callbackControl newFunction)
{
	currentCallbackControl = newFunction;
}

void BipedGaitClass::detach()
{
	currentCallbackControl = NULL;
}

void BipedGaitClass::setMotor(int id, int initValue, float angle2ValueRatio, int direction)
{
	_motorInitValue[id] = initValue;
	_motorDirection[id] = direction;
	_angle2ValueRatio[id] = angle2ValueRatio;
}

int BipedGaitClass::getMotor(int id)
{
	return _motorInitValue[id] + (int)(_jointAngle[id] * _angle2ValueRatio[id] * _motorDirection[id]);
}

float BipedGaitClass::getAngle(int id)
{
	return _jointAngle[id];
}

void BipedGaitClass::setKinematic(float hipWidth, float thighLength, float calfLength)
{
	_hipWidth = hipWidth;
	_thighLength = thighLength;
	_calfLength = calfLength;
	_legLength = thighLength + calfLength;

	_thighLength2 = thighLength * thighLength;
	_calfLength2 = calfLength * calfLength;
}

void BipedGaitClass::setPose(float legLengthRatio, float hipAngle)
{
	legLengthRatio = clamp(legLengthRatio, 0.6, 1);
	hipAngle = clamp(hipAngle, -30, 30);

	_hipHeight = _legLength * legLengthRatio;
	_hipAngle = hipAngle;
}

void BipedGaitClass::tuneSwing(float cycleTime, float dspRatio, float hipSwingRatio, float footSwingRatio, float pelvisSwingRatio)
{
	cycleTime = clamp(cycleTime, 0.1, 5);
	dspRatio = clamp(dspRatio, 0, 1);	
	hipSwingRatio = clamp(hipSwingRatio, 0.1, 1);
	footSwingRatio = clamp(footSwingRatio, 0, 1);
	pelvisSwingRatio = clamp(pelvisSwingRatio, 0, 1);

	_cycleTime = cycleTime;
	_dspTime1 = _cycleTime * dspRatio * 0.5;
	_dspTime2 = _cycleTime - _dspTime1;

	_hipSwing = _hipWidth * hipSwingRatio;
	_footHeightSwing = _calfLength * footSwingRatio;
	_pelvisSwing = 30 * pelvisSwingRatio;
}

void BipedGaitClass::balance(float roll, float pitch)
{
	roll = clamp(roll, -30, 30);
	pitch = clamp(pitch, -30, 30);

	_balanceRoll = roll;
	_balancePitch = pitch;
}

void BipedGaitClass::control(float forward, float right, float turn)
{
	forward = clamp(forward, -_hipWidth, _hipWidth);
	right = clamp(right, -_hipWidth, _hipWidth);
	turn = clamp(turn, -45, 45);

	_footFBswing = forward * 0.5;
	_footRLswing = right * 0.5;
	_footTurnSwing = turn * 0.5;
}

void BipedGaitClass::process()
{
	unsigned long time = micros();
	unsigned long delta = time - _preTime;

	if(_preTime == 0)
		process(0);
	else
		process((float)delta / 1000000);
	
	_preTime = time;
}

void BipedGaitClass::process(float deltaTime)
{
	for(int i=0; i<NUM_JOINT; i++)
		_jointAngle[i] = 0;
	
	float angle1, angle2;

	// Adjust hip angle offset
	_jointAngle[R_HIP_PITCH] += _hipAngle;
	_jointAngle[L_HIP_PITCH] -= _hipAngle;

	// Adjust balance
	float r_foot_height_offset = 0;
	float l_foot_height_offset = 0;
	if(_balanceRoll != 0)
	{		
		_jointAngle[R_ANKLE_ROLL] += _balanceRoll;
		_jointAngle[L_ANKLE_ROLL] += _balanceRoll;
		if(_balanceRoll > 0)
			l_foot_height_offset -= _hipWidth * tan(_balanceRoll * _deg2Rad);
		else
			r_foot_height_offset -= _hipWidth * tan(-_balanceRoll * _deg2Rad);
	}
	if(_balancePitch != 0)
	{
		angle1 = _balancePitch * 0.5;
		_jointAngle[R_HIP_PITCH] -= angle1;
		_jointAngle[L_HIP_PITCH] += angle1;
		_jointAngle[R_ANKLE_PITCH] -= angle1;
		_jointAngle[L_ANKLE_PITCH] += angle1;
	}
	
	// Compute swing
	float r_foot_height_swing = 0;
	float l_foot_height_swing = 0;
	float r_foot_fb_swing = 0;
	float l_foot_fb_swing = 0;
	float r_foot_rl_swing = 0;
	float l_foot_rl_swing = 0;
	float r_foot_turn_swing = 0;
	float l_foot_turn_swing = 0;
	if(_phase !=  STOP_RIGHT_STEP && _phase !=  STOP_LEFT_STEP)
	{
		if(_time >= _cycleTime)
		{
			_time -= _cycleTime;
			if(_phase == START_RIGHT_STEP)
			{
				if(_stop == true)
					_phase = END_LEFT_STEP;
				else
					_phase = LOOP_LEFT_STEP;
			}
			else if(_phase == START_LEFT_STEP)
			{
				if(_stop == true)
					_phase = END_RIGHT_STEP;
				else
					_phase = LOOP_RIGHT_STEP;
			}
			else if(_phase == LOOP_RIGHT_STEP)
			{
				if(_stop == true)
					_phase = END_LEFT_STEP;
				else
					_phase = LOOP_LEFT_STEP;
			}
			else if(_phase == LOOP_LEFT_STEP)
			{
				if(_stop == true)
					_phase = END_RIGHT_STEP;
				else
					_phase = LOOP_RIGHT_STEP;
			}
			else if(_phase == END_RIGHT_STEP)
				_phase = STOP_RIGHT_STEP;
			else if(_phase == END_LEFT_STEP)
				_phase = STOP_LEFT_STEP;
		}
		else
			_time += deltaTime;

		float offset1, offset2;
		float t = _time / _cycleTime;
		switch(_phase)
		{
		case START_RIGHT_STEP:
			// hip swing			
			offset1 = _hipSwing * sin(M_PI * t);
			r_foot_rl_swing += offset1;
			l_foot_rl_swing += offset1;

			if(_time < _dspTime1)
			{
			}
			else if(_time >= _dspTime1 && _time <= _dspTime2)
			{
				t = (_time - _dspTime1) / (_dspTime2 - _dspTime1);

				// foot height swing
				offset1 = sin(M_PI * t);
				r_foot_height_swing += (offset1 *_footHeightSwing);

				// pelvis swing
				_jointAngle[R_HIP_ROLL] -= (offset1 * _pelvisSwing);

				// foot fb swing
				offset1 = _footFBswing * sin(_m_pi_2 * t);
				r_foot_fb_swing += offset1;
				l_foot_fb_swing += -offset1;

				// foot rl swing
				if(_footRLswing > 0)
				{
					offset1 = _footRLswing * sin(_m_pi_2 * t);
					r_foot_rl_swing += offset1;
					l_foot_rl_swing += -offset1;
				}

				// foot turn swing
				if(_footTurnSwing > 0)
				{
					offset1 = _footTurnSwing * sin(_m_pi_2 * t);
					if(_footFBswing >= 0)
					{
						r_foot_turn_swing += -offset1;
						l_foot_turn_swing += offset1;
					}
					else
					{
						r_foot_turn_swing -= -offset1;
						l_foot_turn_swing -= offset1;
					}
				}
			}
			else if(_time > _dspTime2)
			{
				// foot fb swing
				r_foot_fb_swing += _footFBswing;
				l_foot_fb_swing += -_footFBswing;

				// foot rl swing
				if(_footRLswing > 0)
				{					
					r_foot_rl_swing += _footRLswing;
					l_foot_rl_swing += -_footRLswing;
				}

				// foot turn swing
				if(_footTurnSwing > 0)
				{
					if(_footFBswing >= 0)
					{
						r_foot_turn_swing += -_footTurnSwing;
						l_foot_turn_swing += _footTurnSwing;
					}
					else
					{
						r_foot_turn_swing -= -_footTurnSwing;
						l_foot_turn_swing -= _footTurnSwing;
					}
				}
			}
			break;

		case START_LEFT_STEP:
			// hip swing
			offset1 = _hipSwing * -sin(M_PI * t);
			r_foot_rl_swing += offset1;
			l_foot_rl_swing += offset1;

			if(_time < _dspTime1)
			{
			}
			else if(_time >= _dspTime1 && _time <= _dspTime2)
			{
				t = (_time - _dspTime1) / (_dspTime2 - _dspTime1);

				// foot height swing
				offset1 = sin(M_PI * t);
				l_foot_height_swing += (offset1 * _footHeightSwing);

				// pelvis swing
				_jointAngle[L_HIP_ROLL] += (offset1 * _pelvisSwing);

				// foot fb swing
				offset1 = _footFBswing * sin(_m_pi_2 * t);
				r_foot_fb_swing += -offset1;
				l_foot_fb_swing += offset1;

				// foot rl swing
				if(_footRLswing < 0)
				{
					offset1 = _footRLswing * sin(_m_pi_2 * t);
					r_foot_rl_swing += -offset1;
					l_foot_rl_swing += offset1;
				}

				// foot turn swing
				if(_footTurnSwing < 0)
				{
					offset1 = _footTurnSwing * sin(_m_pi_2 * t);
					if(_footFBswing >= 0)
					{
						r_foot_turn_swing += offset1;
						l_foot_turn_swing += -offset1;
					}
					else
					{
						r_foot_turn_swing -= offset1;
						l_foot_turn_swing -= -offset1;
					}
				}
			}
			else if(_time > _dspTime2)
			{
				// foot fb swing
				r_foot_fb_swing += -_footFBswing;
				l_foot_fb_swing += _footFBswing;

				// foot rl swing
				if(_footRLswing < 0)
				{
					r_foot_rl_swing += -_footRLswing;
					l_foot_rl_swing += _footRLswing;
				}

				// foot turn swing
				if(_footTurnSwing < 0)
				{
					if(_footFBswing >= 0)
					{
						r_foot_turn_swing += _footTurnSwing;
						l_foot_turn_swing += -_footTurnSwing;
					}
					else
					{
						r_foot_turn_swing -= _footTurnSwing;
						l_foot_turn_swing -= -_footTurnSwing;
					}
				}
			}
			break;

		case LOOP_RIGHT_STEP:
			// hip swing
			offset1 = _hipSwing * sin(M_PI * t);
			r_foot_rl_swing += offset1;
			l_foot_rl_swing += offset1;

			if(_time < _dspTime1)
			{
				// foot fb swing
				r_foot_fb_swing += -_footFBswing;
				l_foot_fb_swing += _footFBswing;

				// foot rl swing
				if(_footRLswing < 0) // left move
				{
					r_foot_rl_swing += -_footRLswing;
					l_foot_rl_swing += _footRLswing;
				}

				// foot turn swing
				if(_footTurnSwing < 0) // left turn
				{
					if(_footFBswing >= 0)
					{
						r_foot_turn_swing += _footTurnSwing;
						l_foot_turn_swing += -_footTurnSwing;
					}
					else
					{
						r_foot_turn_swing -= _footTurnSwing;
						l_foot_turn_swing -= -_footTurnSwing;
					}
				}
			}
			else if(_time >= _dspTime1 && _time <= _dspTime2)
			{
				t = (_time - _dspTime1) / (_dspTime2 - _dspTime1);

				// foot height swing
				offset1 = sin(M_PI * t);
				r_foot_height_swing += (offset1 * _footHeightSwing);

				// pelvis swing
				_jointAngle[R_HIP_ROLL] -= (offset1 * _pelvisSwing);

				// foot fb swing
				offset1 = _footFBswing * cos(M_PI * t);
				r_foot_fb_swing += -offset1;
				l_foot_fb_swing += offset1;

				// foot rl swing
				if(_footRLswing > 0) // right move
				{
					offset1 = _footRLswing * sin(_m_pi_2 * t);
					r_foot_rl_swing += offset1;
					l_foot_rl_swing += -offset1;
				}
				else if(_footRLswing < 0) // left move
				{
					offset1 = -_footRLswing * cos(_m_pi_2 * t);
					r_foot_rl_swing += offset1;
					l_foot_rl_swing += -offset1;
				}

				// foot turn swing
				if(_footTurnSwing > 0) // right turn
				{
					offset1 = _footTurnSwing * sin(_m_pi_2 * t);
					if(_footFBswing >= 0)
					{
						r_foot_turn_swing += -offset1;
						l_foot_turn_swing += offset1;
					}
					else
					{
						r_foot_turn_swing -= -offset1;
						l_foot_turn_swing -= offset1;
					}
				}
				else if(_footTurnSwing < 0) // left turn
				{
					offset1 = -_footTurnSwing * cos(_m_pi_2 * t);
					if(_footFBswing >= 0)
					{
						r_foot_turn_swing += -offset1;
						l_foot_turn_swing += offset1;
					}
					else
					{
						r_foot_turn_swing -= -offset1;
						l_foot_turn_swing -= offset1;
					}
				}
			}
			else if(_time > _dspTime2)
			{
				// foot fb swing
				r_foot_fb_swing += _footFBswing;
				l_foot_fb_swing += -_footFBswing;

				// foot rl swing
				if(_footRLswing > 0) // right move
				{
					r_foot_rl_swing += _footRLswing;
					l_foot_rl_swing += -_footRLswing;
				}

				// foot turn swing
				if(_footTurnSwing > 0) // left turn
				{
					if(_footFBswing >= 0)
					{
						r_foot_turn_swing += -_footTurnSwing;
						l_foot_turn_swing += _footTurnSwing;
					}
					else
					{
						r_foot_turn_swing -= -_footTurnSwing;
						l_foot_turn_swing -= _footTurnSwing;
					}
				}
			}
			break;

		case LOOP_LEFT_STEP:
			// hip swing
			offset1 = _hipSwing * -sin(M_PI * t);
			r_foot_rl_swing += offset1;
			l_foot_rl_swing += offset1;

			if(_time < _dspTime1)
			{
				// foot fb swing
				r_foot_fb_swing += _footFBswing;
				l_foot_fb_swing += -_footFBswing;

				// foot rl swing
				if(_footRLswing > 0) // right move
				{
					r_foot_rl_swing += _footRLswing;
					l_foot_rl_swing += -_footRLswing;
				}

				// foot turn swing
				if(_footTurnSwing > 0) // left turn
				{
					if(_footFBswing >= 0)
					{
						r_foot_turn_swing += -_footTurnSwing;
						l_foot_turn_swing += _footTurnSwing;
					}
					else
					{
						r_foot_turn_swing -= -_footTurnSwing;
						l_foot_turn_swing -= _footTurnSwing;
					}
				}
			}
			else if(_time >= _dspTime1 && _time <= _dspTime2)
			{
				t = (_time - _dspTime1) / (_dspTime2 - _dspTime1);

				// foot height swing
				offset1 = sin(M_PI * t);
				l_foot_height_swing += (offset1 * _footHeightSwing);

				// pelvis swing
				_jointAngle[L_HIP_ROLL] += (offset1 * _pelvisSwing);

				// foot fb swing
				offset1 = _footFBswing * cos(M_PI * t);
				r_foot_fb_swing += offset1;
				l_foot_fb_swing += -offset1;

				// foot rl swing
				if(_footRLswing > 0) // right move
				{
					offset1 = _footRLswing * cos(_m_pi_2 * t);
					r_foot_rl_swing += offset1;
					l_foot_rl_swing += -offset1;
				}
				else if(_footRLswing < 0) // left move
				{
					offset1 = -_footRLswing * sin(_m_pi_2 * t);
					r_foot_rl_swing += offset1;
					l_foot_rl_swing += -offset1;
				}

				// foot turn swing
				if(_footTurnSwing > 0) // right turn
				{
					offset1 = _footTurnSwing * cos(_m_pi_2 * t);
					if(_footFBswing >= 0)
					{
						r_foot_turn_swing += -offset1;
						l_foot_turn_swing += offset1;
					}
					else
					{
						r_foot_turn_swing -= -offset1;
						l_foot_turn_swing -= offset1;
					}
				}
				else if(_footTurnSwing < 0) // left turn
				{
					offset1 = -_footTurnSwing * sin(_m_pi_2 * t);
					if(_footFBswing >= 0)
					{
						r_foot_turn_swing += -offset1;
						l_foot_turn_swing += offset1;
					}
					else
					{
						r_foot_turn_swing -= -offset1;
						l_foot_turn_swing -= offset1;
					}
				}
			}
			else if(_time > _dspTime2)
			{
				// foot fb swing
				r_foot_fb_swing += -_footFBswing;
				l_foot_fb_swing += _footFBswing;

				// foot rl swing
				if(_footRLswing < 0) // left move
				{
					r_foot_rl_swing += -_footRLswing;
					l_foot_rl_swing += _footRLswing;
				}

				// foot turn swing
				if(_footTurnSwing < 0) // left turn
				{
					if(_footFBswing >= 0)
					{
						r_foot_turn_swing += _footTurnSwing;
						l_foot_turn_swing += -_footTurnSwing;
					}
					else
					{
						r_foot_turn_swing -= _footTurnSwing;
						l_foot_turn_swing -= -_footTurnSwing;
					}
				}
			}
			break;

		case END_RIGHT_STEP:
			// hip swing
			offset1 = _hipSwing * sin(M_PI * t);
			r_foot_rl_swing += offset1;
			l_foot_rl_swing += offset1;

			if(_time < _dspTime1)
			{
				// foot fb swing
				r_foot_fb_swing += -_footFBswing;
				l_foot_fb_swing += _footFBswing;

				// foot rl swing
				if(_footRLswing < 0) // left move
				{
					r_foot_rl_swing += -_footRLswing;
					l_foot_rl_swing += _footRLswing;
				}

				// foot turn swing
				if(_footTurnSwing < 0) // left turn
				{
					if(_footFBswing >= 0)
					{
						r_foot_turn_swing += _footTurnSwing;
						l_foot_turn_swing += -_footTurnSwing;
					}
					else
					{
						r_foot_turn_swing -= _footTurnSwing;
						l_foot_turn_swing -= -_footTurnSwing;
					}
				}
			}
			else if(_time >= _dspTime1 && _time <= _dspTime2)
			{
				t = (_time - _dspTime1) / (_dspTime2 - _dspTime1);

				// foot height swing
				offset1 = sin(M_PI * t);
				r_foot_height_swing += (offset1 * _footHeightSwing);

				// pelvis swing
				_jointAngle[R_HIP_ROLL] -= (offset1 * _pelvisSwing);

				// foot fb swing
				offset1 = _footFBswing * cos(_m_pi_2 * t);
				r_foot_fb_swing += -offset1;
				l_foot_fb_swing += offset1;

				// foot rl swing
				if(_footRLswing < 0) // left move
				{
					offset1 = -_footRLswing * cos(_m_pi_2 * t);
					r_foot_rl_swing += offset1;
					l_foot_rl_swing += -offset1;
				}

				// foot turn swing
				if(_footTurnSwing < 0) // left turn
				{
					offset1 = -_footTurnSwing * cos(_m_pi_2 * t);
					if(_footFBswing >= 0)
					{
						r_foot_turn_swing += -offset1;
						l_foot_turn_swing += offset1;
					}
					else
					{
						r_foot_turn_swing -= -offset1;
						l_foot_turn_swing -= offset1;
					}
				}
			}
			else if(_time > _dspTime2)
			{
			}
			break;

		case END_LEFT_STEP:
			// hip swing
			offset1 = _hipSwing * -sin(M_PI * t);
			r_foot_rl_swing += offset1;
			l_foot_rl_swing += offset1;

			if(_time < _dspTime1)
			{
				// foot fb swing
				r_foot_fb_swing += _footFBswing;
				l_foot_fb_swing += -_footFBswing;

				// foot rl swing
				if(_footRLswing > 0) // right move
				{
					r_foot_rl_swing += _footRLswing;
					l_foot_rl_swing += -_footRLswing;
				}

				// foot turn swing
				if(_footTurnSwing > 0) // left turn
				{
					if(_footFBswing >= 0)
					{
						r_foot_turn_swing += -_footTurnSwing;
						l_foot_turn_swing += _footTurnSwing;
					}
					else
					{
						r_foot_turn_swing -= -_footTurnSwing;
						l_foot_turn_swing -= _footTurnSwing;
					}
				}
			}
			else if(_time >= _dspTime1 && _time <= _dspTime2)
			{
				t = (_time - _dspTime1) / (_dspTime2 - _dspTime1);

				// foot height swing
				offset1 = sin(M_PI * t);
				l_foot_height_swing += (offset1 * _footHeightSwing);

				// pelvis swing
				_jointAngle[L_HIP_ROLL] += (offset1 * _pelvisSwing);

				// foot fb swing
				offset1 = _footFBswing * cos(_m_pi_2 * t);
				r_foot_fb_swing += offset1;
				l_foot_fb_swing += -offset1;

				// foot rl swing
				if(_footRLswing > 0) // right move
				{
					offset1 = _footRLswing * cos(_m_pi_2 * t);
					r_foot_rl_swing += offset1;
					l_foot_rl_swing += -offset1;
				}

				// foot turn swing
				if(_footTurnSwing > 0) // right turn
				{
					offset1 = _footTurnSwing * cos(_m_pi_2 * t);
					if(_footFBswing >= 0)
					{
						r_foot_turn_swing += -offset1;
						l_foot_turn_swing += offset1;
					}
					else
					{
						r_foot_turn_swing -= -offset1;
						l_foot_turn_swing -= offset1;
					}
				}
			}
			else if(_time > _dspTime2)
			{
			}
			break;
		}
	}
	
	// Compute foot height offset
	float r_hip_height = _hipHeight + r_foot_height_offset - r_foot_height_swing;
	if(r_hip_height != _legLength)
	{
		angle1 = acos((_calfLength2 - r_hip_height * r_hip_height - _thighLength2) / (-2 * r_hip_height * _thighLength)) * _rad2Deg;
		angle2 = acos((_thighLength2 - r_hip_height * r_hip_height - _calfLength2) / (-2 * r_hip_height * _calfLength)) * _rad2Deg;		
		_jointAngle[R_HIP_PITCH] -= angle1;
		_jointAngle[R_ANKLE_PITCH] -= angle2;
		_jointAngle[R_KNEE_PITCH] += (angle1 + angle2);
	}
	float l_hip_height = _hipHeight + l_foot_height_offset - l_foot_height_swing;
	if(l_hip_height != _legLength)
	{
		angle1 = acos((_calfLength2 - l_hip_height * l_hip_height - _thighLength2) / (-2 * l_hip_height * _thighLength)) * _rad2Deg;
		angle2 = acos((_thighLength2 - l_hip_height * l_hip_height - _calfLength2) / (-2 * l_hip_height * _calfLength)) * _rad2Deg;		
		_jointAngle[L_HIP_PITCH] += angle1;
		_jointAngle[L_ANKLE_PITCH] += angle2;
		_jointAngle[L_KNEE_PITCH] -= (angle1 + angle2);
	}

	// Compute foot F/B offset
	if(r_foot_fb_swing != 0)
	{
		angle1 = asin(r_foot_fb_swing / r_hip_height) * _rad2Deg;
		_jointAngle[R_HIP_PITCH] -= angle1;
		_jointAngle[R_ANKLE_PITCH] += angle1;
	}
	if(l_foot_fb_swing != 0)
	{
		angle1 = asin(l_foot_fb_swing / l_hip_height) * _rad2Deg;
		_jointAngle[L_HIP_PITCH] += angle1;
		_jointAngle[L_ANKLE_PITCH] -= angle1;
	}

	// Compute foot R/L offset
	if(r_foot_rl_swing != 0)
	{
		angle1 = asin(r_foot_rl_swing / r_hip_height) * _rad2Deg;
		_jointAngle[R_HIP_ROLL] -= angle1;
		_jointAngle[R_ANKLE_ROLL] += angle1;
	}
	if(l_foot_rl_swing != 0)
	{
		angle1 = asin(l_foot_rl_swing / l_hip_height) * _rad2Deg;
		_jointAngle[L_HIP_ROLL] -= angle1;
		_jointAngle[L_ANKLE_ROLL] += angle1;
	}

	// Compute foot turn offset
	_jointAngle[R_HIP_YAW] += r_foot_turn_swing;
	_jointAngle[L_HIP_YAW] += l_foot_turn_swing;	
	
	if(currentCallbackControl)
		(*currentCallbackControl)();
}

//******************************************************************************
//* Private Methods
//******************************************************************************
float BipedGaitClass::clamp(float value, float min, float max)
{
	if(value < min)
		value = min;
	else if(value > max)
		value = max;

	return value;
}

BipedGaitClass BipedGait;


