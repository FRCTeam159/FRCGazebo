#include "DriveWithJoystick.h"
#include "Subsystems/DriveTrain.h"
#include "RobotMap.h"

DriveWithJoystick::DriveWithJoystick()
{
	// Use Requires() here to declare subsystem dependencies
	Requires(driveTrain.get());
}

// Called just before this Command runs the first time
void DriveWithJoystick::Initialize()
{
	std::cout << "DriveWithJoystick::Initialize()" << std::endl;
}

// Called repeatedly when this Command is scheduled to run
void DriveWithJoystick::Execute()
#define MINTHRESHOLD 0.3
#define MINOUTPUT 0.0
{
	// Get axis values
	Joystick *stick = oi->GetJoystick();

	if (stick->GetRawButton(LOWGEAR_BUTTON)){
		driveTrain->SetLowGear();
	}
	else if(stick->GetRawButton(HIGHGEAR_BUTTON)){
		driveTrain->SetHighGear();
	}
#if JOYTYPE == XBOX_GAMEPAD
	float yAxis=-stick->GetRawAxis(4); // left stick - drive
	float xAxis=-stick->GetRawAxis(1); // right stick - rotate
	float zAxis = 0;

#else
	float yAxis = stick-> GetY();
	float xAxis = stick-> GetX();
	float zAxis = stick-> GetZ();
	// Run axis values through deadband
#endif
	yAxis = quadDeadband(MINTHRESHOLD, MINOUTPUT, yAxis);
	xAxis = quadDeadband(MINTHRESHOLD, MINOUTPUT, xAxis);
	zAxis = quadDeadband(MINTHRESHOLD, MINOUTPUT, zAxis);
	driveTrain->CustomArcade(xAxis, yAxis, zAxis,true);
	//visionSubsystem->Process();

}

// Make this return true when this Command no longer needs to run execute()
bool DriveWithJoystick::IsFinished()
{
	return false;
}

// Called once after isFinished returns true
void DriveWithJoystick::End()
{
	std::cout << "DriveWithJoystick Finished" << std::endl;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveWithJoystick::Interrupted()
{
	End();
}
float DriveWithJoystick::quadDeadband(float minThreshold, float minOutput, float input)
{
	if (input > minThreshold) {
		return ((((1 - minOutput)
				/ ((1 - minThreshold)* (1 - minThreshold)))
				* ((input - minThreshold)* (input - minThreshold)))
				+ minOutput);
	} else {
		if (input < (-1 * minThreshold)) {
			return (((minOutput - 1)
					/ ((minThreshold - 1)* (minThreshold - 1)))
					* ((minThreshold + input)* (minThreshold + input)))
					- minOutput;
		}

		else {
			return 0;
		}
	}
}
