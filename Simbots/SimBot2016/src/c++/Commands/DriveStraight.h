/*
 * DriveStraight.h
 *
 *  Created on: Mar 3, 2016
 *      Author: alpiner
 */

#ifndef SRC_COMMANDS_DRIVESTRAIGHT_H_
#define SRC_COMMANDS_DRIVESTRAIGHT_H_
#include <WPILib.h>
#include <Commands/Command.h>

class DriveStraight: public Command
{
protected:
    static double distance;
    static double heading;
	bool at_position;
	bool at_heading;
	static double distance_error;
	static double angle_error;
public:
	DriveStraight(double d, double h);
	void Initialize();
	bool IsFinished();
	void Execute();
	void End();
	void Interrupted();

	class AngleControl: public PIDSource, public PIDOutput{
	private:
		PIDController pid;
		double target;
	public:
		AngleControl(double P, double I, double D);
		double PIDGet();
		void PIDWrite(double d);
		bool AtTarget();
		void Initialize(double d);
		void End();

	};
	class DistanceControl: public PIDSource, public PIDOutput{
	private:
		PIDController pid;
		double target;

	public:
		DistanceControl(double P, double I, double D);
		double PIDGet();
		void PIDWrite(double d);
		bool AtTarget();
		void Initialize(double d);
		void End();
	};
	AngleControl Acntrl;
	DistanceControl Dcntrl;
};

#endif /* SRC_COMMANDS_DRIVESTRAIGHT_H_ */
