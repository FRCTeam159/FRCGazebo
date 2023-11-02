/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import frc.robot.Robot;

public class DriveToTarget extends CommandBase {
  NetworkTable table;
  double targetDistance = 12.0;
  //private static final double kP = 0.02;
  //private static final double minError = kP * 3;
  private double error = 0;
  private PIDController pid;

  private static final double P = 0.1;
	private static final double I = 0.02;
	private static final double D = 0.0;
  private static final double TOL = 0.05;

  public DriveToTarget(double d) {
    pid = new PIDController(P, I, D);
    targetDistance = d;
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    table = inst.getTable("TargetData");
    System.out.println("DriveToTarget.initialize");
    pid.setTolerance(1.0);
    pid.reset();
    pid.setSetpoint(targetDistance);
    //SmartDashboard.putNumber("RangeError", 0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    //double range = table.getEntry("Range").getDouble(0.0);
    double s = Robot.driveTrain.getDistance();
    double d = pid.calculate(s, targetDistance);
    Robot.driveTrain.set(d, d);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
    //return error < minError? true : false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted){
    System.out.println("DriveToTarget.end");
  }
}
