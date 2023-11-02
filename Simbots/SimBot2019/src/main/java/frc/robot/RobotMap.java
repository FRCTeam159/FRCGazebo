package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public interface RobotMap {
	public static int RIGHTWHEELS = 1;
	public static int LEFTWHEELS = 2;

	public static final int FRONT_CLIMBER_MOTOR = 4;
	public static final int BACK_CLIMBER_MOTOR = 5;

	public static final int GRABBER_MOTOR = 6; // MAXbot: 7
	public static final int ELEVATOR_MOTOR = 7; // MAXbot: 5

	// Piston IDs
	public static final int GRABBER_PISTON_ID = 0;
	public static final int ELEVATOR_PISTON_ID = 1;
	public static final int PISTON_FORWARD = 2;
	public static final int PISTON_REVERSE = 3;
	// Servo IDs
	public static final int ARM_SERVO = 0;

	// Button axis IDs
	public static final int LEFT_JOYSTICK = 1; // MAXbot: 1
	public static final int RIGHT_JOYSTICK = 4; // MAXbot: 4
	public static final int LEFT_TRIGGER = 2;
	public static final int RIGHT_TRIGGER = 5;
	public static final int HATCH_CARGO_MODE = 0;  // MAXbot: 6
	public static final int EMERGENCY_STOP = 4;    // MAXbot: 7

	// Button IDs
	public static final int LEFT_TRIGGER_BUTTON = 4;
	public static final int RIGHT_TRIGGER_BUTTON = 5;
	public static final int FRONT_CLIMBER_BUTTON = 8;
	public static final int BACK_CLIMBER_BUTTON = 7;

	public static final int ELEVATOR_TILT_BUTTON = 9; // left 
	public static final int GRABBER_TILT_BUTTON = 10; // right 
	public static final int INTAKE_BUTTON = 1;
	public static final int ELEVATOR_RESET_HEIGHT_BUTTON = 2;
	public static final int OUTPUT_BUTTON = 3;
	public static final int ARMS_TOGGLE_BUTTON = 4;

	// General Constants
	public static final int TIMEOUT = 10;
	public static final int ENCODER_TIMEOUT = 10;
	public static final int ENCODER_WINDOW_SIZE = 4;
	public static final int ENCODER_STATUS_FRAME_PERIOD = 4;

	public static final int LEFT_POSITION = 0;
	public static final int CENTER_POSITION = 1;
	public static final int RIGHT_POSITION = 2;
	public static final int ILLEGAL_POSITION = 3;

	public static int STICK = 0;

	public static int LOWGEAR_BUTTON = 4;
	public static int HIGHGEAR_BUTTON = 3;

	public static int GEARSHIFTID = 0;

}
