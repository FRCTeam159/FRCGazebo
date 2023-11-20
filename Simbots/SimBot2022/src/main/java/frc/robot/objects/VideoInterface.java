// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.objects;

import org.opencv.core.Mat;

/** Add your docs here. */
public interface VideoInterface {
    Mat getFrame();
    void process();
    boolean isConnected();
}
