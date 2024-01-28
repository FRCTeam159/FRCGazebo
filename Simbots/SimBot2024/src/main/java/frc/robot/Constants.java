// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public interface Constants {

    public static String chnlnames[] = { "FL", "FR", "BL", "BR" };

    // motor ids
    public static final int FL_ID = 1;
    public static final int FL_DRIVE = 1;
    public static final int FL_TURN = 2;

    public static final int FR_ID = 2;
    public static final int FR_DRIVE = 3;
    public static final int FR_TURN = 4;

    public static final int BL_ID = 3;
    public static final int BL_DRIVE = 5;
    public static final int BL_TURN = 6;

    public static final int BR_ID = 4;
    public static final int BR_DRIVE = 7;
    public static final int BR_TURN = 8;

    public static final int ARM = 9;
    public static final int PICKUP = 10;
    public static final int SHOOTER = 11;
    public static final int PUSHER = 12;

    public static final int ARM_LIMIT = 2;

    public static double PICKUP_ANGLE=-10;
    public static double SPEAKER_SHOOT_ANGLE=8;
    public static double AMP_SHOOT_ANGLE=95;
  
}

