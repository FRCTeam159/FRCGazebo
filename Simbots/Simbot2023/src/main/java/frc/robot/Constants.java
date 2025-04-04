// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    
    public static boolean kTestMode=false;

    // Drivetrain parameters

    public static double kMaxVelocity = 3; // 4 meters per second
    public static double kMaxAcceleration = 1; // meters/second/second
    public static double kMaxAngularSpeed =  Math.toRadians(360); // 3 rotation per second
    public static double kMaxAngularAcceleration = Math.toRadians(360); // 1 rotations/s/s
    public static final double kWheelRadius = Units.inchesToMeters(2.0); // 2 inches
    public static final double kFrontWheelBase = 18.625; // 0.472 distance bewteen front wheels (in)
	public static final double kSideWheelBase =  31;  // 0.787 distance b18eteen side wheels (in)

    // Image parameters
    public static final int kImageWidth = 640;
    public static final int kImageHeight = 480;

    public static final double kBoxTargetArea = 17;
    public static final double kConeTargetArea = 12;
    public static final double kPostTargetArea = 2;

    // Arm parameters
    public static final double kStageOneLength = Units.inchesToMeters(43.18); // 1.0968
    public static final double kStageTwoLength = Units.inchesToMeters(30.59); // 0.7742+
    public static final double kStageOneAngleOffset = Math.toRadians(105.4); // starting angle
    public static final double kStageTwoAngleOffset = Math.toRadians(201);
    public static final double kRotateAngleOffset = Math.toRadians(-21); // starting angle
    
    public static final double kMaxLowerArmAngularSpeed=Math.toRadians(45);
    public static final double kMaxLowerArmAngularAcceleration=Math.toRadians(20);
    public static final double kMaxUpperArmAngularSpeed=Math.toRadians(360);
    public static final double kMaxUpperArmAngularAcceleration=Math.toRadians(180);
    public static final double kMaxWristRotateAngularSpeed=Math.toRadians(360);
    public static final double kMaxWristRotaterAcceleration=Math.toRadians(180);
    
    // field parameters
    
    // CAN IDS
    // Drivetrain 
    public static final int kFl_Drive = 1;
    public static final int kFl_Turn = 2;
    public static final int kFr_Drive = 3;
    public static final int kFr_Turn = 4;
    public static final int kBr_Drive = 7;
    public static final int kBr_Turn = 8;
    public static final int kBl_Drive = 5;
    public static final int kBl_Turn = 6;

    // Arm
    public static final int kStageOneChannel = 9;
    public static final int kStageTwoChannel = 10;
    public static final int kWristRotateChannel = 11;
    public static final int kWristTwistChannel = 12;
  
    public static final int kClawMotorID1 = 15;
    public static final int kClawMotorID2 = 14;

    // public static final int 

    public static final Object[][] modes = { //1st letter is operation P-pickup, D-Dropoff; 2nd letter is type C-cone, B-box; 3rd letter is location B-bottom, M-middle T-top
        {"PCB", 0, 10, 2}, //modes[x][1] is y-target on camera; modes[x][2] is area-target; modes[x][3] is target type (refer to limelight subsys)
        {"PCM", 0, 10, 2},
        {"PBB", 0, 10, 1},
        {"PBM", 0, 10, 1},
        {"DCB", 0, 10, 3},
        {"DCM", 0, 10, 3},
        {"DCT", 0, 10, 3},
        {"DBB", 0, 10, 0},
        {"DBM", 0, 10, 0},
        {"DBT", 0, 10, 0},
    };
}
