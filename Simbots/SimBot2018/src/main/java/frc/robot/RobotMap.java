package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public interface RobotMap {
  public static final int FRONTLEFT = 1;
  public static final int FRONTRIGHT = 2;
 
  public static final int CUBEWHEELS = 5;
  public static final int ELEVATORMOTOR = 7;

  //public static final int STICK = 0;
  public static final int GEAR_TOGGLE_BUTTON = 4;
  public static final int ARM_TOGGLE_BUTTON = 1;
  public static final int CUBE_PUSH_BUTTON = 2;
  public static final int CUBE_GRAB_BUTTON = 3;


  public static final int LEFTTRIGGER = 2;
  public static final int RIGHTTRIGGER = 3;

  public static final int GEARSHIFTID = 0;
  public static final int RESET_ELEVATOR_BUTTON = 5;
  public static final int ELEVATOR_GO_TO_SWITCH_BUTTON = 6;

  public static final boolean SQUARE_INPUTS = true;
 

}
