#include "UltraUpdate.h"

UltraUpdate::UltraUpdate() {
	// Use Requires() here to declare subsystem dependencies
	Requires(ultrasonicSubsystem.get());
	std::cout << "new UltraUpdate"<< std::endl;
}

// Called just before this Command runs the first time
void UltraUpdate::Initialize() {
	ultrasonicSubsystem->Enable();
	std::cout << "UltraUpdate Started .."<< std::endl;
}

// Called repeatedly when this Command is scheduled to run
void UltraUpdate::Execute() {
	ultrasonicSubsystem->GetDistance();
}

// Make this return true when this Command no longer needs to run execute()
bool UltraUpdate::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void UltraUpdate::End() {
	ultrasonicSubsystem->Disable();
	std::cout << "UltraUpdate End"<< std::endl;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void UltraUpdate::Interrupted() {
	End();
}
