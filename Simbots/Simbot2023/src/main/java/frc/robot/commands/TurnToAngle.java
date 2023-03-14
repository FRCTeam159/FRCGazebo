// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.Constants.*;

public class TurnToAngle extends CommandBase {
  boolean debug=false;
  boolean turning=false;
  double target_angle=0;
  double heading=0;
  Drivetrain m_drive;
  double start_phase=0;
  double last_heading;
  double correction=0;
  //double direction=1;
  double start_time=0;
  

  final ProfiledPIDController m_turningPIDController= 
  new ProfiledPIDController(0.25,0.0,0,
    new TrapezoidProfile.Constraints(
        Math.toDegrees(0.25*kMaxAngularSpeed),Math.toDegrees(0.25*kMaxAngularAcceleration)));

  public TurnToAngle(Drivetrain drive, double angle) {
    m_drive=drive;
    m_turningPIDController.setTolerance(2.0,2);
    target_angle=angle;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("TurnToAngle.initialize");

    turning=true;
    start_phase=m_drive.getHeading();
    heading=last_heading=0;
    
    //target_angle=180*direction;
    m_turningPIDController.reset(0);
    
    start_time=m_drive.getClockTime();
    if(debug)    
      System.out.format("turn started current:%-3.1f target:%-3.1f %s\n",
      start_phase,target_angle+start_phase,target_angle>0?"CW":"CCW");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    calculate();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("TurnToAngle.end");

  }

  // Returns true when the command should end.
  // note: for a profiled PID controller "atSetpoint()" doesn't work (BUG)
  // reason 
  // - ProfiledPIDController contains a regular PIDcontroller (m_controller)
  //   ProfiledPIDController.atSetpoint() is passed to m_controller.atSetpoint().
  //   but m_controller.setSetpoint() function is never called so a private
  //   variable m_controller.m_haveSetpoint is never set to true and m_controller.atSetpoint()
  //   always returns false
  // - also another BUG exists in ProfiledPIDController.getPositionError() (see below)
  @Override
  public boolean isFinished() {
    if(!turning)
        return true;
    // BUG double perr=m_turningPIDController.getPositionError();
    //- another problem is that at the start of the trapazoid ramp m_turningPIDController.getPositionError()
    //  returns a value close to zero if the control parameter also starts at zero so it can't be used in a
    //  check for "atSetpoint"
    //- the error check needs to return the error based on the difference between the user's calculate target and
    //  the current value of the control parameter
    heading= m_drive.getHeading()-start_phase;
    double perr=heading-target_angle;
    double verr=m_turningPIDController.getVelocityError();
    double ptol=m_turningPIDController.getPositionTolerance();
    double vtol=m_turningPIDController.getVelocityTolerance();
    double delt=m_drive.getClockTime()-start_time;
    if(delt>12){
      System.out.println("Turn aborted - timeout expired");
      turning=false;
      return true;
    }
    if(Math.abs(perr)<ptol && Math.abs(verr)<vtol){
      if(debug)
        System.out.format("atTurnSetpoint time:%-2.1f heading:%-3.1f perr:%-3.1f verr:%-2.1f\n",delt,heading+start_phase,perr,verr);
      turning=false;
      return true;
    }
    return false;   
  }
  // return the calculated rotation value (right stick)
  public double calculate(){  
    heading=m_drive.getHeading()-start_phase;
    correction=0;
    if(last_heading !=heading){ 
      double err=heading-target_angle;
      correction=m_turningPIDController.calculate(heading,target_angle);
      if(debug)
        System.out.format("   heading:%-3.1f target:%-3.1f err:%-3.1f cor:%-1.2f\n",
          heading+start_phase,target_angle+start_phase,err,correction);     
      if(isFinished()){
        if(debug)
          System.out.println("turning finished - at setpoint");
        turning=false;
      }
    }
    last_heading=heading;
    return correction;  
  }
}
