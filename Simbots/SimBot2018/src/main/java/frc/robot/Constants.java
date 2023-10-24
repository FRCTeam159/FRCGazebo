package frc.robot;

public interface Constants {
  // robot position
  public static final int POSITION_CENTER = 0;
  public static final int POSITION_LEFT = 1;
  public static final int POSITION_RIGHT = 2;

  // target object
  public static final int OBJECT_SWITCH = 0;
  public static final int OBJECT_SCALE = 1;
  public static final int OBJECT_NONE = 2;

 // auto options
  
  public static final int CENTER_SWITCH = 0;
  public static final int SAME_SCALE = 1;
  public static final int SAME_SWITCH = 2;
  public static final int OTHER_SCALE = 3;
  public static final int GO_STRAIGHT = 4;
  public static final int TWO_CUBE_SIDE = 5;
  public static final int TWO_CUBE_CENTER = 6;

  // grabber states
  public static final int DROP = 0;
  public static final int HOLD = 1;
  public static final int PUSH = 2;
  public static final int GRAB = 3;
  public static final int OPEN = 4;
  public static final int CLOSE = 5;
}
