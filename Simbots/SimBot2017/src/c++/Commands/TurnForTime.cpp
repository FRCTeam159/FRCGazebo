#include <Commands/TurnForTime.h>

TurnForTime::TurnForTime(double t, double s) {
	time = t;
	speed = s;
	Requires(driveTrain.get());
	std::cout << "new TurnForTime"<< std::endl;
}

// Called just before this Command runs the first time
void TurnForTime::Initialize() {
	std::cout << "TurnForTime Started("<<time<<","<<speed<<")"<< std::endl;
	driveTrain->EnableDrive();
	targetTime = Timer::GetFPGATimestamp() + time;
}

// Called repeatedly when this Command is scheduled to run
void TurnForTime::Execute() {
	driveTrain->TankDrive(speed, -speed);
}

// Make this return true when this Command no longer needs to run execute()
bool TurnForTime::IsFinished() {
	currentTime = Timer::GetFPGATimestamp();
	if(currentTime >= targetTime)
		return true;
	return false;
}

// Called once after isFinished returns true
void TurnForTime::End() {
	std::cout << "TurnForTime End"<< std::endl;
	driveTrain->DisableDrive();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void TurnForTime::Interrupted() {
	End();
}
