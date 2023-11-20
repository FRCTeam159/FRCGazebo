// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TurnToAngle extends CommandBase {
  DriveTrain m_drive;
  double m_angle;
  final PIDController m_controller=new PIDController(0.3,0,0.1);

  public TurnToAngle(DriveTrain drive, double angle) { 
    m_drive=drive;
    m_angle=angle;
  }
  // =================================================
  // initialize: Called when the command is initially scheduled.
  // =================================================
  @Override
  public void initialize() {
    System.out.println("TurnToAngle.initialize");
    m_controller.enableContinuousInput(-200, 200);
    m_controller.setTolerance(0.5, 0.1); // TurnToleranceDeg, TurnRateToleranceDegPerS 
  }
  // =================================================
  // execute: Called every time the scheduler runs while the command is scheduled
  // =================================================
  @Override
  public void execute() {
    double heading=m_drive.getHeading();
    double correction=m_controller.calculate(heading,m_angle);
    m_drive.arcadeDrive(0, -correction);  
  }
  // =================================================
  // isFinished: Returns true when the command should end
  // =================================================
  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    //return m_controller.atGoal();
    return m_controller.atSetpoint();
  }
  // =================================================
  // end: Called once the command ends or is interrupted.
  // =================================================
  @Override
  public void end(boolean interrupted) {
    System.out.println("TurnToAngle.end");
    m_drive.reset();
  }
}
