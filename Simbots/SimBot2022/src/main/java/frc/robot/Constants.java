// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */


public interface Constants {

    // motor ids
    public static final int FRONT_LEFT = 3;
    public static final int BACK_LEFT = 4;
    public static final int FRONT_RIGHT = 2;
    public static final int BACK_RIGHT = 1;
    public static final int CLIMBER = 5;
    public static final int INTAKE = 7;
    public static final int SHOOTER = 8;

    // xbox controller keys

    public static final int LEFT_JOYSTICK = 1;
    public static final int RIGHT_JOYSTICK = 4;

    public static final int LEFT_BUMPER = 5;
	public static final int RIGHT_BUMPER = 6;

    public static final int LEFT_TRIGGER = 2;
	public static final int RIGHT_TRIGGER = 3;

}
