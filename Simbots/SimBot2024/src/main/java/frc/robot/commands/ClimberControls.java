// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberControls extends Command {
  /** Creates a new ClimberControls. 
 * @param controller 
 * @param climber */
  Climber m_climber;
  XboxController m_controller;

  public ClimberControls(Climber climber, XboxController controller) {
    m_climber=climber;
    m_controller=controller;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climber.disable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int pov=m_controller.getPOV();
    switch(pov){
      default:
        m_climber.disable();
        break;
      case 180: 
        m_climber.climbDown();
      break;
      case 90: 
        m_climber.climbToTarget();
      break;
      case 0: 
        m_climber.climbUp();
      break;
      case 270: 
        m_climber.hookChain();
      break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
