/*
  BipedGait.h - BipedGait library
  Copyright (C) 2014 Robolink.  All rights reserved.
*/

#ifndef BipedGait_h
#define BipedGait_h

#include "Arduino.h"

#define NUM_JOINT		12

// define joint id
#define R_HIP_YAW		0
#define R_HIP_ROLL		1
#define R_HIP_PITCH		2
#define R_KNEE_PITCH	3
#define R_ANKLE_PITCH	4
#define R_ANKLE_ROLL	5
#define L_HIP_YAW		6
#define L_HIP_ROLL		7
#define L_HIP_PITCH		8
#define L_KNEE_PITCH	9
#define L_ANKLE_PITCH	10
#define L_ANKLE_ROLL	11


extern "C"
{
// callback function types
    typedef void (*callbackControl)(void);		
}


class BipedGaitClass
{
public:
	BipedGaitClass();

	void start();
	void stop();
	void process();
	void process(float deltaTime); // unit: sec
    void attach(callbackControl newFunction);
	void detach();

	void setMotor(int id, int initValue, float angle2ValueRatio, int direction);
	int getMotor(int id);
	float getAngle(int id);
	void setKinematic(float hipWidth, float thighLength, float calfLength); // unit: cm
	void setPose(float legLengthRatio, float hipAngle);
	void tuneSwing(float cycleTime, float dspRatio, float hipSwingRatio, float footSwingRatio, float pelvisSwingRatio);
	void balance(float roll, float pitch); // unit: degree
	void control(float forward, float right, float turn); // unit: cm, degree

private:
	float _jointAngle[NUM_JOINT];	
	int _motorInitValue[NUM_JOINT];
	int _motorDirection[NUM_JOINT];
	float _angle2ValueRatio[NUM_JOINT];

	float _hipWidth;
	float _hipAngle;
	float _thighLength;
	float _thighLength2;
	float _calfLength;
	float _calfLength2;
	float _legLength;
	float _balanceRoll;
	float _balancePitch;
	float _hipHeight;
	float _hipSwing;
	float _pelvisSwing;
	float _footHeightSwing;
	float _footFBswing;
	float _footRLswing;
	float _footTurnSwing;

	boolean _stop;
	float _cycleTime;
	float _time;
	float _dspTime1;
	float _dspTime2;
	int _phase;
	unsigned long _preTime;

	float _deg2Rad;
	float _rad2Deg;
	float _m_pi_2;

	
/* callback functions */
    callbackControl currentCallbackControl;

/* private methods ------------------------------ */
	float clamp(float value, float min, float max);
};

extern BipedGaitClass BipedGait;

#endif /*BipedGait_h */

