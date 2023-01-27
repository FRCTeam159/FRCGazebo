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
  double start_time;
  double start_pos=0;
  double last_pos=0;

  static boolean debug=true;
  
 final PIDController m_controller=new PIDController(0.1,0.0,0.0);

  public TurnToAngle(Drivetrain drive, double angle) { 
    m_drive=drive;
    m_angle=angle;
    addRequirements(drive);
  }
  // =================================================
  // initialize: Called when the command is initially scheduled.
  // =================================================
  @Override
  public void initialize() {
    System.out.println("TurnToAngle.initialize");
    //m_controller.enableContinuousInput(-180, 180);
    m_controller.setTolerance(0.5, 1.0); // TurnToleranceDeg, TurnRateToleranceDegPerS 
    m_drive.startAuto();
    start_pos=m_drive.getHeading();

   //m_drive.driveForward(0);
    last_time=start_time=m_drive.getClockTime();
    // need a dummy read from controller to avoid bad first correction (looks like a bug
    m_controller.setSetpoint(m_angle);

    m_controller.calculate(0,m_angle);
  }
  // =================================================
  // execute: Called every time the scheduler runs while the command is scheduled
  // =================================================
  @Override
  public void execute() {
    double heading=m_drive.getHeading();
    double tm=m_drive.getClockTime();

    if ((tm-last_time)>0.015 && heading !=last_pos){
      double pos=heading-start_pos;
      double correction=0.2*m_controller.calculate(pos,m_angle);
      if(debug)
      System.out.format("time:%f dt:%d gain:%g pos:%g pos error:%g vel error:%g\n",
        tm-start_time,(int)(1000*(tm-last_time)),correction,pos,m_controller.getPositionError(),m_controller.getVelocityError());
     //m_drive.turnInPlace(correction);
      m_drive.drive(0.0, 0,correction,true);
      //m_drive.turnInPlace(correction);

    }
    last_pos=heading;
    last_time=tm;
  }
  // =================================================
  // isFinished: Returns true when the command should end
  // =================================================
  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return (last_time>0.05 && m_controller.atSetpoint()) || m_drive.disabled();
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
