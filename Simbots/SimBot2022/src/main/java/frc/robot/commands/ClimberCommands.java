// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class ClimberCommands extends CommandBase implements Constants {
  /**
   * Creates a new ClimberCommands.
   * 
   * @param climber
   * @param controller
   */

  XboxController m_controller;
  Climber m_climber;

  public ClimberCommands(XboxController controller, Climber climber) {
    m_climber = climber;
    m_controller = controller;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climber.enable();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int direction = m_controller.getPOV(0);
    switch (direction) {
      case 0:
        m_climber.setLifterUp();
        break;
      case 180:
        m_climber.setLifterDown();
        break;
      case 90:
        m_climber.armsOut();
        break;
      case 270:
        m_climber.armsIn();
        break;
    }
    double left = m_controller.getRawAxis(LEFT_TRIGGER);
    double right = m_controller.getRawAxis(RIGHT_TRIGGER);
    if (left > 0.2) {
      m_climber.setLifterDown(left);
    } else if (right > 0.2) {
      m_climber.setLifterUp(right);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
