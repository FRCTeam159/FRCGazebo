// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Arm;

public class Pickup extends Command implements Constants{
  private final Arm m_arm;
  boolean note_captured=false;

  public Pickup(Arm arm) {
    m_arm=arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Pickup.init");
    m_arm.setPickupOn();
    Robot.status="Pickup";
    note_captured=false;
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Arm.noteAtIntake())
      note_captured=true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Pickup.end");
    m_arm.setPickupOff();
    m_arm.setTargetAngle(SPEAKER_SHOOT_ANGLE); // lift note off the ground
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {   
    if(note_captured && Arm.noteAtShooter()){
      return true;
    }
    return false;
  }
}
