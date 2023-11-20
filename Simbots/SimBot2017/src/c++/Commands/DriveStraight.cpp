#include "DriveStraight.h"

#define P 0.05
#define I 0.0
#define D 0.0

DriveStraight::DriveStraight(double d)  : CommandBase("DriveStraight"),
	pid(P,I,D,this,this)
{
	Requires(driveTrain.get());
	distance=d;
    std::cout << "new DriveStraight("<<d<<")"<< std::endl;
}

// Called just before this Command runs the first time
void DriveStraight::Initialize() {
	driveTrain->Reset();
  	pid.Reset();
	pid.SetSetpoint(distance);
	pid.SetAbsoluteTolerance(1);
	pid.Enable();
	driveTrain->Enable();
	std::cout << "DriveStraight Started .."<< std::endl;
}

// Called repeatedly when this Command is scheduled to run
void DriveStraight::Execute() {
}

// Make this return true when this Command no longer needs to run execute()
bool DriveStraight::IsFinished() {
	return pid.OnTarget();
}

// Called once after isFinished returns true
void DriveStraight::End() {
	pid.Disable();
    std::cout << "DriveStraight End" << std::endl;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveStraight::Interrupted() {
	End();
}

double DriveStraight::PIDGet() {
	double s=driveTrain->GetDistance();
#ifdef DEBUG_COMMAND
    std::cout << "DriveStraight::PIDGet("<<s<<")"<< std::endl;
#endif
	return s;
}

void DriveStraight::PIDWrite(double d) {
#ifdef DEBUG_COMMAND
    std::cout << "DriveStraight::PIDWrite("<<d<<")"<< std::endl;
#endif
	driveTrain->TankDrive(d,d);
}
