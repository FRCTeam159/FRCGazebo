#include "CloseGate.h"

CloseGate::CloseGate() {
	Requires(gearSubsystem.get());
    std::cout << "new CloseGate"<< std::endl;
}

// Called just before this Command runs the first time
void CloseGate::Initialize() {
    std::cout << "CloseGate Started ..." << std::endl;
    gearSubsystem->Close();
}

// Called repeatedly when this Command is scheduled to run
void CloseGate::Execute() {

}

// Make this return true when this Command no longer needs to run execute()
bool CloseGate::IsFinished() {
	return !gearSubsystem->IsOpen();
}

// Called once after isFinished returns true
void CloseGate::End() {
    std::cout << "CloseGate End" << std::endl;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void CloseGate::Interrupted() {
	End();
}
