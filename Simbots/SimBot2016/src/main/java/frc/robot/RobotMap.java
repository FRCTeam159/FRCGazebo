package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public interface RobotMap {

	// game controller Button axis IDs
	public static final int LEFT_JOYSTICK = 1; // MAXbot: 1
	public static final int RIGHT_JOYSTICK = 4; // MAXbot: 4
	public static final int LEFT_TRIGGER = 2;
	public static final int RIGHT_TRIGGER = 5;

	// game controller Button IDs
	public static final int LEFT_BUMPER_BUTTON = 4;
	public static final int RIGHT_BUMPER_BUTTON = 5;
	public static final int RIGHT_BUTTON = 3;
	public static final int LEFT_BUTTON = 1;
	public static final int UP_BUTTON = 4;
	public static final int DOWN_BUTTON = 2;

	public static final int A_BUTTON = DOWN_BUTTON;
	public static final int B_BUTTON = RIGHT_BUTTON;
	public static final int X_BUTTON = LEFT_BUTTON;
	public static final int Y_BUTTON = UP_BUTTON;

	// button mapping to function
    public static final int MODE_TOGGLE_BUTTON = LEFT_BUTTON;

	public static final int STEP_UP_BUTTON = RIGHT_BUMPER_BUTTON;
	public static final int STEP_DOWN_BUTTON = LEFT_BUMPER_BUTTON;

	// mode = SHOOTING
	public static final int SHOOT_BALL_BUTTON = UP_BUTTON;
	public static final int GATE_TOGGLE_BUTTON = DOWN_BUTTON;
	public static final int FW_TOGGLE_BUTTON = RIGHT_BUTTON;
	
	// mode = LOADING
	public static final int LOAD_TOGGLE_BUTTON = UP_BUTTON;
	public static final int ROLLERS_TOGGLE_BUTTON = DOWN_BUTTON;
		 // bind commands based on current mode
    // case 1: SHOOTER mode

    // upBtnCmnd1.WhenPressed(new ShootBall());
    // rightBtnCmnd1.WhenPressed(new StepShooterAngle(10));
    // leftBtnCmnd1.WhenPressed(new StepShooterAngle(-10));
    // downBtnCmnd1.WhenPressed(new ToggleGate());

    // case 1: LOADER mode
    // upBtnCmnd2.WhenPressed(new ToggleLoad());
    // rightBtnCmnd2.WhenPressed(new StepLoaderAngle(5));
    // leftBtnCmnd2.WhenPressed(new StepLoaderAngle(-5));
    // downBtnCmnd2.WhenPressed(new ToggleRollers());

	// Robot field position 
	
	public static final int LEFT_POSITION = 0;
	public static final int CENTER_POSITION = 1;
	public static final int RIGHT_POSITION = 2;
	public static final int ILLEGAL_POSITION = 3;
	
	// gyro IDs

	public static final int FRAME_GYRO = 0;
	public static final int SHOOTER_GYRO = 1;
	public static final int LOADER_GYRO = 2;

	// motor IDs

	public static final int RIGHTWHEELS = 1;
	public static final int LEFTWHEELS = 2;
	public static final int SHOOTER_MOTOR = 3;
	public static final int PUSHWHEEL_MOTOR = 4;
	public static final int FLYWHEEL_MOTOR = 5;
	public static final int SHOOTER_GATE_MOTOR = 6;
	public static final int LOADER_ARMS_MOTOR = 7;
	public static final int ROLLER_MOTOR = 8;

	// limit switch  IDs

	public static final int SHOOTER_SWITCH = 2;
	public static final int SHOOTER_GATE_SWITCH = 6;
	public static final int LOADER_ARMS_SWITCH = 7;

    // rangefinder IDs

	public static final int SHOOTER_RANGEFINDER = 2;
	public static final int FRAME_RANGEFINDER = 1;

	// Piston IDs
	

}
