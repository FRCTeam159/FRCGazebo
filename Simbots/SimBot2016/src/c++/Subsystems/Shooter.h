/*
 * Shooter.h
 *
 *  Created on: Feb 19, 2016
 *      Author: alpiner
 */

#ifndef SRC_SUBSYSTEMS_SHOOTER_H_
#define SRC_SUBSYSTEMS_SHOOTER_H_

#include "WPILib.h"
#include "CANTalon.h"

class Shooter: public Subsystem , public PIDSource, public PIDOutput {
	CANTalon angleMotor;
	CANTalon leftFWMotor;
	CANTalon rightFWMotor;
	AnalogGyro accel;

	PIDController *angle_pid;

	double angle,max_angle,min_angle;
	double max_angle_error;
	double flywheel_target;
	double flywheel_speed;
	void Init();
	bool initialized;

	void InitDefaultCommand();
	void GoToLowerLimitSwitch();

public:
	Shooter();
	~Shooter();
	void SetTargetAngle(double a);
	void SetTargetSpeed(double a);
	double GetTargetAngle();
	double GetMaxAngle() { return max_angle;}
	double GetMinAngle() { return min_angle;}
	void Disable();
	void DisableFlywheels();
	void EnableFlywheels();

	bool IsAtAngle();
	bool IsAtSpeed();
	bool AtLowerLimit();

	double GetTargetSpeed();
	double GetFWSpeed();
	double GetAngle();

	void AutonomousInit();
	void TeleopInit();
	void DisabledInit();
	void Log();
	void LogSpeed(double d);
	void LogAngle(double d);

	bool IsInitialized();
	void Initialize();
	void SetInitialized();
	void EnableAngleController(bool b);

	void Execute();
	void Reset();
    double PIDGet();

	void PIDWrite(double output);
    void SetAngle(double output);

	void SetMaxAngleError(double d);
    void SetDefaultMaxAngleError();


};

#endif /* SRC_SUBSYSTEMS_SHOOTER_H_ */
