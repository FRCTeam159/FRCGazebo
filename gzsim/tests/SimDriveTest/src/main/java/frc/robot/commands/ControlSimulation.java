// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Simulation;

public class ControlSimulation extends CommandBase {
  Simulation m_simulation;
  private boolean resetting=false;

  /** Creates a new SimControl. */
  public ControlSimulation(Simulation subsystem) {
    m_simulation=subsystem;
    addRequirements(subsystem);
    SmartDashboard.putBoolean("Reset", false);
  }

  public void reset(){
    m_simulation.reset();
    resetting=true;
  }
  
  public void enable(){
    m_simulation.enable();
    resetting=false;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("ControlSimulation started");
     enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean b=SmartDashboard.getBoolean("Reset", resetting);
    if(resetting && !b)
      enable();
    else if (!resetting && b)
      reset(); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("ControlSimulation end");
    m_simulation.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
