// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.VisionProcess;

public class AdjustShot extends CommandBase {
  int ntargets = 0;
  double timeout;
  double start_time;
  double elapsed;
  double overshoot=4;
  boolean at_setpoint=false;

  static final double TURN_RATE = 0.2;
  static final double ANGLE_RATE = 1;

  PIDController hpid = new PIDController(0.3, 0.00, 0);
  PIDController vpid = new PIDController(1, 0.01, 0);

  /** Creates a new AdjustShot. */
  public AdjustShot(double t) {
    timeout = t;
    hpid.setTolerance(0.25, 0.5);
    vpid.setTolerance(0.8, 0.5);
    addRequirements(Robot.driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ntargets = VisionProcess.getNumTargets();
    start_time = Robot.getTime();
    if(ntargets>0)
    System.out.println("AdjustShot started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elapsed = Robot.getTime() - start_time;
    ntargets = VisionProcess.getNumTargets();
    if (ntargets > 0 && !at_setpoint) {
      double hoffset = VisionProcess.getHOffset();
      double voffset = VisionProcess.getVOffset();
      double herr = -hpid.calculate(hoffset, 0);
      double verr = vpid.calculate(voffset, overshoot);
      double vangle = Robot.shooter.getShooterAngle();
      double nangle=vangle + ANGLE_RATE*verr;
      Robot.shooter.setTargetAngle(nangle);
      //Robot.shooter.adjustAngle();
      Robot.driveTrain.arcadeDrive(0, TURN_RATE * herr);
      //Robot.driveTrain.drive(0, TURN_RATE * herr);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(at_setpoint){
      System.out.println("AdjustShot end - at target");
      return true;
    }
    if (ntargets==0){
      System.out.println("AdjustShot end - no target");
      return true;
    }
    if (elapsed >= timeout){
      System.out.println("AdjustShot end - timeout expired");
      return true;
    }
    at_setpoint=hpid.atSetpoint() && vpid.atSetpoint();
    return false;
  }
}
