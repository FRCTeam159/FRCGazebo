#include "Commands/DriveForTime.h"

#include "../CommandBase.h"
#include "WPILib.h"

DriveForTime::DriveForTime(double t, double s)
{
	Requires(driveTrain.get());
	std::cout << "new DriveForTime"<< std::endl;
	time = t;
	speed = s;
}

// Called just before this Command runs the first time
void DriveForTime::Initialize()
{
	std::cout << "DriveForTime Started("<<time<<","<<speed<<")"<< std::endl;
	driveTrain->EnableDrive();
	targetTime = Timer::GetFPGATimestamp() + time;
}

// Called repeatedly when this Command is scheduled to run
void DriveForTime::Execute()
{
	driveTrain->TankDrive(speed, speed);
}

// Make this return true when this Command no longer needs to run execute()
bool DriveForTime::IsFinished()
{
	currentTime = Timer::GetFPGATimestamp();
	if(currentTime >= targetTime)
		return true;
	return false;
}

// Called once after isFinished returns true
void DriveForTime::End()
{
	std::cout << "DriveForTime End"<< std::endl;
	driveTrain->DisableDrive();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveForTime::Interrupted()
{
	End();
}
