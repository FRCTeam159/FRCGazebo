#include "Subsystems/ClimbingSubsystem.h"
#include "../RobotMap.h"
#include "Commands/Climb.h"


ClimbingSubsystem::ClimbingSubsystem() : Subsystem("ClimbingSubsystem"),climberMotor(CLIMBING_MOTOR) {
	frc::SmartDashboard::PutNumber("climberVoltage", 0.1);
	frc::SmartDashboard::PutNumber("climberCurrentThreshold", 10);
}

void ClimbingSubsystem::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
	SetDefaultCommand(new Climb());
}

bool ClimbingSubsystem::IsAtTop() { // TODO
	double currentThreshold = SmartDashboard::GetNumber("climberCurrentThreshold", 10);

	if(GetCurrent() > currentThreshold){
		return true;
	} else {
		return false;
	}
}

double ClimbingSubsystem::GetCurrent(){
	return climberMotor.GetOutputCurrent();
}

void ClimbingSubsystem::Stop() {
	climberMotor.Set(0);
}

void ClimbingSubsystem::ClimberClimb() {
	climberMotor.Set(SmartDashboard::GetNumber("climberVoltage", 0.1));
}
// Put methods for controlling this subsystem
// here. Call these from Commands.
