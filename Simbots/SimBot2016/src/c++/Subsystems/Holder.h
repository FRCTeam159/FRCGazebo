/*
 * Holder.h
 *
 *  Created on: Feb 19, 2016
 *      Author: alpiner
 */

#ifndef SRC_SUBSYSTEMS_HOLDER_H_
#define SRC_SUBSYSTEMS_HOLDER_H_
#include "WPILib.h"
#include "CANTalon.h"


class Holder: public Subsystem {
	CANTalon gateMotor;
	CANTalon pushMotor;
	AnalogInput ballSensor;
	bool found_zero=false;

	bool initialized=false;
	double push_hold_speed;
	bool pushRequested=false;
	double gateTicksPerRevolution;

	void Init();
	void InitDefaultCommand();

public:
	Holder();
	bool IsGateOpen();
	bool IsGateClosed();
	bool IsBallPresent();
	void OpenGate();
	void CloseGate();
	void PushBall();
	void HoldBall();
	void RemoveBall();

	void AutonomousInit();
	void TeleopInit();
	void DisabledInit();

	bool IsInitialized();
	void SetInitialized();
	void Initialize();
	void Reset();
	void Log();
	void Execute();
	bool FindZero();
	void SetPushHoldSpeed(double d);
	bool PushRequested() { return pushRequested;}
};

#endif /* SRC_SUBSYSTEMS_HOLDER_H_ */
