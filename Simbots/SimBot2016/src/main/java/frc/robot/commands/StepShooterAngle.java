// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class StepShooterAngle extends CommandBase {
  double angle;
  double timeout=0;
  double start_time=0;
  double elapsed;
  boolean debug=true;

  /** Creates a new StepShooterAngle. */
  public StepShooterAngle(double t, double a) {
    angle = a;
    timeout=t;
    elapsed=0;
    addRequirements(Robot.shooter);
  }

  void debugPrint(String msg) {
    if (debug)
      System.out.format("%1.2f StepAngle %s\n",elapsed,msg);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    debugPrint("started target=" + angle);
    Robot.shooter.setTargetAngle(angle);
    start_time=Robot.simulation.getSimTime();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elapsed=Robot.simulation.getSimTime()-start_time;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (elapsed>=timeout)
      debugPrint("end - timeout expired");
    else
      debugPrint("end - shooter angle at target");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (elapsed>=timeout) 
      return true;
    return Robot.shooter.isAtAngle();
    
  }
}
