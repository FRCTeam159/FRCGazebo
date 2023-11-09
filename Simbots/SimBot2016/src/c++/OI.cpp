/*
 * OI.cpp
 *
 *  Created on: Jun 3, 2014
 *      Author: alex
 */

#include "OI.h"
#include "Commands/ToggleMode.h"
#include "Commands/ShootBall.h"
#include "Commands/StepShooterAngle.h"
#include "Commands/StepLoaderAngle.h"
#include "Commands/ToggleGate.h"
#include "Commands/ToggleRollers.h"
#include <Commands/ToggleLoad.h>
#include <Commands/AdjustShot.h>

int OI::mode=SHOOTING;


OI::OI() :
		rightBtnCmnd1(this),
		rightBtnCmnd2(this),
		leftBtnCmnd1(this),
		leftBtnCmnd2(this),
		upBtnCmnd1(this),
		upBtnCmnd2(this),
		downBtnCmnd1(this),
		downBtnCmnd2(this)
{

	joy= new Joystick(0);

    // Create some buttons
    d_right = new JoystickButton(joy, 2);
    d_left= new JoystickButton(joy, 3);
    d_down= new JoystickButton(joy, 1);
    d_up = new JoystickButton(joy, 4);

    d_mode = new JoystickButton(joy, 5);
    d_target = new JoystickButton(joy, 6);

    // bind down button to mode toggle

    d_target->ToggleWhenPressed(new AdjustShot());
    d_mode->ToggleWhenPressed(new ToggleMode());

    // bind commands based on current mode
    // case 1: SHOOTER mode
    upBtnCmnd1.WhenPressed(new ShootBall());
    rightBtnCmnd1.WhenPressed(new StepShooterAngle(10));
    leftBtnCmnd1.WhenPressed(new StepShooterAngle(-10));
    downBtnCmnd1.WhenPressed(new ToggleGate());

    // case 1: LOADER mode
    upBtnCmnd2.WhenPressed(new ToggleLoad());
    rightBtnCmnd2.WhenPressed(new StepLoaderAngle(5));
    leftBtnCmnd2.WhenPressed(new StepLoaderAngle(-5));
    downBtnCmnd2.WhenPressed(new ToggleRollers());

    SmartDashboard::PutData("ToggleGate",new ToggleGate());

}

Joystick* OI::GetJoystick() {
	return joy;
}

