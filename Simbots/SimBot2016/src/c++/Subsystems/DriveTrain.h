#ifndef DriveTrain_H
#define DriveTrain_H

#include "WPILib.h"
#include "CANTalon.h"


/**
 * The DriveTrain subsystem incorporates the sensors and actuators attached to
 * the robots chassis. These include four drive motors, a left and right encoder
 * and a gyro.
 */
class DriveTrain : public Subsystem {
	CANTalon left_motor;
	CANTalon right_motor;
private:
	AnalogGyro gyro;
	double x_deadband=0.2;
	double y_deadband=0.2;
	double cpr=1;
	bool inverted=false;
	bool squared_inputs=true;
	bool disabled=true;
	bool pid_disabled=true;
	double target_distance=0;
	double target_angle=0;
	bool reverse_driving=false;

	void DisablePID();
	void EnablePID();
	void Limit(double &num);
	void SquareInputs(double &left, double &right);
	void SetDistance(double d);

public:
	DriveTrain();

	void InitDefaultCommand();
	void SetSquaredInputs(double d){ squared_inputs=d;}
	void Log();
	void Reset();
	void Enable();
	void Disable();
	void EndTravel();

	void Drive(Joystick* joy);
	double GetHeading();

	void TeleopInit();
	void AutonomousInit();
	void DisabledInit();

	double GetDistance();
	double GetSpeed();

	double GetLeftDistance();
	double GetRightDistance();
	double GetLeftSpeed();
	double GetRightSpeed();
	double GetLeftVoltage();
	double GetRightVoltage();

	virtual double Deadband(double x, double ignore);
	virtual void SetDeadband(double x, double y);
	virtual void SetDistancePerPulse(double diam,double ticks, bool b);
	virtual double GetDistancePerPulse();
	//virtual void SetPID(int mode, double P, double I, double D);
	virtual void Turn(double d);

	virtual void Drive(double l,double r);
	virtual void SetInverted(bool b) { inverted=b;}

	virtual bool IsDisabled() { return disabled;}
	virtual bool IsPIDDisabled() { return pid_disabled;}
	virtual bool IsInverted() { return inverted;}
	virtual void SetReverseDriving(bool b);
	virtual bool ReverseDriving();

};

#endif
