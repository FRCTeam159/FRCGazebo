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

	// Piston IDs
	public static final int PISTON_FORWARD = 2;
	public static final int PISTON_REVERSE = 3;
	// Servo IDs

	// Button axis IDs
	public static final int LEFT_JOYSTICK = 1; // MAXbot: 1
	public static final int RIGHT_JOYSTICK = 4; // MAXbot: 4
	public static final int LEFT_TRIGGER = 2;
	public static final int RIGHT_TRIGGER = 5;

	// Button IDs
	public static final int LEFT_TRIGGER_BUTTON = 4;
	public static final int RIGHT_TRIGGER_BUTTON = 5;
	
	public static final int INTAKE_BUTTON = 1;
	public static final int OUTPUT_BUTTON = 3;

	public static final int GEARTOGGLEBUTTON=2;
	public static final int FUELPUSHERBUTTON=1;
	public static final int CLIMBERBUTTON=3;


	// General Constants
	
	public static final int LEFT_POSITION = 0;
	public static final int CENTER_POSITION = 1;
	public static final int RIGHT_POSITION = 2;
	public static final int ILLEGAL_POSITION = 3;

	public static int STICK = 0;

	public static int LOWGEAR_BUTTON = 4;
	public static int HIGHGEAR_BUTTON = 3;

	public static int GEARSHIFTID = 0;

}
