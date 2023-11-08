#include "FuelMonitor.h"
#include "RobotMap.h"
#define FORWARDVOLTAGE 1
#define REVERSEVOLTAGE -0.5

FuelMonitor::FuelMonitor() {
	Requires(fuelSubsystem.get());
	std::cout << "new FuelMonitor"<< std::endl;
}

// Called just before this Command runs the first time
void FuelMonitor::Initialize() {
	state = WAITFORBUTTON;
	cout << "FuelMonitor Started .."<<endl;
}

// Called repeatedly when this Command is scheduled to run
void FuelMonitor::Execute() {
	Joystick *stick = oi->GetJoystick();
	switch(state){
	default:
	case WAITFORBUTTON:
		if(stick->GetRawButton(FUELPUSHERBUTTON)){
			cout << "FuelMonitor Push Started .."<<endl;
			state=WAITFORUPPERLIMIT;
		}
		else
			fuelSubsystem->SetVoltage(0);
		break;
	case WAITFORUPPERLIMIT:
		if(fuelSubsystem->AtUpperLimit())
			state= WAITFORLOWERLIMIT;
		else
			fuelSubsystem->SetVoltage(FORWARDVOLTAGE);
		break;
	case WAITFORLOWERLIMIT:
		if(fuelSubsystem->AtLowerLimit()){
			cout << "FuelMonitor Push Finished"<<endl;
			state= WAITFORBUTTON;
		}
		else
			fuelSubsystem->SetVoltage(REVERSEVOLTAGE);
		break;
	}
}

// Make this return true when this Command no longer needs to run execute()
bool FuelMonitor::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void FuelMonitor::End() {
	cout << "FuelMonitor End"<<endl;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void FuelMonitor::Interrupted() {
	End();
}
