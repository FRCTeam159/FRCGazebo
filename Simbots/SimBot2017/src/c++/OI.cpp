#include <Commands/GearPlateToggle.h>
#include <Commands/DeliverGear.h>

#include "OI.h"
#include "RobotMap.h"
#include <WPILib.h>


OI::OI() {
	stick = new Joystick(STICK);
	// Process operator interface input here.

	JoystickButton* btn = new JoystickButton(stick, GEARTOGGLEBUTTON);
	btn->ToggleWhenPressed(new GearPlateToggle());
	btn = new JoystickButton(stick, AUTOTARGET_BUTTON);
	btn->ToggleWhenPressed(new DeliverGear());

}

Joystick* OI::GetJoystick() {
	return stick;
}
