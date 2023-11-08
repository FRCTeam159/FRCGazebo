#include "Autonomous.h"
#include "Commands/VisionUpdate.h"

Autonomous::Autonomous() {
	AddParallel(new VisionUpdate());
}
