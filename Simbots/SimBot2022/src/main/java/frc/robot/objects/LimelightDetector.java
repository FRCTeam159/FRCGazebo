// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.objects;

import edu.wpi.first.networktables.NetworkTable;

/** Add your docs here. */
public class LimelightDetector extends TargetDetector
{ 
    String url="http://limelight.local:5802/?action=stream";
    public LimelightDetector(){
    }
   
}
