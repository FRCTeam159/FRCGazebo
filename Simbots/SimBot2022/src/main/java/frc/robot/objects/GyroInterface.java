// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.objects;

/** Add your docs here. */
public interface GyroInterface {
    void enable();
    void disable();
    void reset();
    double getHeading();
    double getRate();
    boolean isEnabled();

}
