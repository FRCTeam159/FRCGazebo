// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class TurnToAngle extends CommandBase {
  Drivetrain m_drive;
  double m_angle;
  double last_time;
  final PIDController m_controller=new PIDController(0.17,1.0,0.07);

  public TurnToAngle(Drivetrain drive, double angle) { 
    m_drive=drive;
    m_angle=angle;
  }
  // =================================================
  // initialize: Called when the command is initially scheduled.
  // =================================================
  @Override
  public void initialize() {
    System.out.println("TurnToAngle.initialize");
    //m_controller.enableContinuousInput(-180, 180);
    m_controller.setTolerance(0.5, 5); // TurnToleranceDeg, TurnRateToleranceDegPerS 
    m_drive.startAuto();
    m_drive.move(0);
    last_time=0;
  }
  // =================================================
  // execute: Called every time the scheduler runs while the command is scheduled
  // =================================================
  @Override
  public void execute() {
    double heading=m_drive.getHeading();
    double tm=m_drive.getTime();
    double correction=m_controller.calculate(heading,m_angle);

     // need a dummy read from controller to avoid bad first correction (looks like a bug)
    if(tm<0.05)
      return;
    if (tm>last_time){
      System.out.format("time:%f dt:%d gain:%g pos:%g pos error:%g vel error:%g\n",
        tm,(int)(1000*(tm-last_time)),correction,heading,m_controller.getPositionError(),m_controller.getVelocityError());
      m_drive.turnInPlace(correction);
    }
    last_time=tm;
  }
  // =================================================
  // isFinished: Returns true when the command should end
  // =================================================
  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return last_time>0.05 && m_controller.atSetpoint();
  }
  // =================================================
  // end: Called once the command ends or is interrupted.
  // =================================================
  @Override
  public void end(boolean interrupted) {
    System.out.println("TurnToAngle.end");
    //m_drive.reset();
  }
}
