/*
 * Assignments.h
 *
 *  Created on: Mar 6, 2016
 *      Author: dean
 */

#ifndef SRC_ASSIGNMENTS_H_
#define SRC_ASSIGNMENTS_H_

enum motorIDs {  // PWM or CAN ids
	DRIVE_LEFT     = 1,
	DRIVE_RIGHT    = 2,
	HOLDER_GATE    = 5,
	SHOOTER_LEFT   = 6,
	HOLDER_PUSH    = 7,
	SHOOTER_RIGHT  = 8,
	SHOOTER_ANGLE  = 9,
	LOADER_ROLLERS = 10,
	LOADER_ANGLE   = 11,
};

enum dioIDs {  // limit switch ids
	GATE_MIN     = 0,
	GATE_MAX     = 1,
	LOADER_MIN   = 2,
	LOADER_MAX   = 3,
	SHOOTER_MIN  = 4,
};

enum AnalogIDs {  // limit switch ids
	SHOOTER_PITCH  = 1, // gyro in simulation
	BALL_SENSOR    = 2, // sonar in simulation
	DRIVE_ANGLE    = 3, // gyro in simulation
	LOADER_PITCH   = 4, // gyro in simulation

};

#define XBOX_GAMEPAD 1
#define EXTREME_3D   2

#ifdef SIMULATION
#define JOYTYPE XBOX_GAMEPAD
#else
#define JOYTYPE EXTREME_3D
#endif

#endif /* SRC_ASSIGNMENTS_H_ */
