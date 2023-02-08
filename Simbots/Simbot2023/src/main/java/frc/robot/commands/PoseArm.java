// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class PoseArm extends CommandBase {
  Arm m_Arm;
  XboxController m_controller;
  
  public PoseArm(Arm arm, XboxController controller) {
    m_Arm = arm;
    m_controller=controller;
    //addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("PoseArm.initialize");
    m_Arm.setHoldingPose();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int direction = m_controller.getPOV(0);
    //m_Arm.posTrim(Units.degreesToRadians(direction));
    //m_Arm.runFeed();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("PoseArm.end:"+interrupted);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
