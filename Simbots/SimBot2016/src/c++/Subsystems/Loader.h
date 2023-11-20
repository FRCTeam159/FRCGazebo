/*
 * Loader.h
 *
 *  Created on: Mar 21, 2016
 *      Author: alpiner
 */

#ifndef SRC_SUBSYSTEMS_LOADER_H_
#define SRC_SUBSYSTEMS_LOADER_H_

#include "WPILib.h"
#include "CANTalon.h"

enum { ROLLERS_OFF,ROLLERS_FORWARD,ROLLERS_REVERSE};

class Loader: public Subsystem, public PIDSource, public PIDOutput {
	CANTalon liftMotor;
	CANTalon rollerMotor;
	PIDController *angle_pid;
	AnalogGyro accel;

	double angle=0;
	double max_angle=80;
	double min_angle=0;
	double roller_speed=0;

	int roller_state=ROLLERS_OFF;
	bool initialized=false;
	bool loading=false;

	void Init();
	void Disable();

	double PIDGet();
	void PIDWrite(double output);

	void InitDefaultCommand();
	void SpinRollers();
public:
	Loader();
	~Loader();
	void Log();

	void SetLifterAngle(double a);
	double GetLifterAngle();
	bool LifterIsAtTargetAngle();
	bool LifterAtLowerLimit();

	void SetRollerState(int b);
	int GetRollorState() { return roller_state;}

	void AutonomousInit();
	void TeleopInit();
	void DisabledInit();

	void GoToZeroLimitSwitch();
	void SetLow();
	void LoadBall();
	bool IsInitialized();
	void SetInitialized();
	void Initialize();
	void Execute();
	void ExecLoad();

	bool Loading();
	void SetLoading(bool b);

};

#endif /* SRC_SUBSYSTEMS_LOADER_H_ */
