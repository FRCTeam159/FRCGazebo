#include "DeliverGear.h"

#include "Commands/DriveForTime.h"
#include "Commands/DriveToTarget.h"
#include "Commands/CloseGate.h"

#define DRIVE_FORWARD 0.3
#define DRIVE_BACKWARD -0.1

DeliverGear::DeliverGear() {
	AddSequential(new DriveToTarget());
	AddSequential(new DriveForTime(1, DRIVE_FORWARD));
	AddSequential(new DriveForTime(1, DRIVE_BACKWARD));
	AddSequential(new CloseGate());
}
