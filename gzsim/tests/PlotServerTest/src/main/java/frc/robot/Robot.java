// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import utils.PlotServer;

/**
 * The VM is configured to automatically run this class. If you change the name of this class or the
 * package after creating this project, you must also update the build.gradle file in the project.
 */
public class Robot extends RobotBase {
  NTServerTest nt_server= new NTServerTest();
  NTClientTest nt_client= new NTClientTest();
  utils.PlotServer plot_server= new PlotServer();
  PlotTest plot_test= new PlotTest();
  public void robotInit() {
    // Dummy project for java compilation (enter "compile" in terminal) 
    // Use Run/Debug menu for testing
  }

  public void disabled() {}
  public void autonomous() {}
  public void teleop() {}
  public void test() {}

  @Override
  public void startCompetition() {
    robotInit();
  }

  @Override
  public void endCompetition() {}
}
