// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gazebo.SimControl;

public class Simulation extends SubsystemBase {
  /** Creates a new SimulationControl. */
  private SimControl m_simcontrol = new SimControl();
  private Drivetrain m_drivetrain;
  private boolean resetting = false;

  public Simulation(Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    SmartDashboard.putBoolean("Reset", false);
  }

  public void reset() {
    m_drivetrain.reset();
    resetting=true;
  }

  public void enable() {
    m_drivetrain.enable();
    resetting=false;
  }

  public void disable() {
    m_drivetrain.disable();
    resetting=false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void simulationPeriodic() {
    boolean b = SmartDashboard.getBoolean("Reset", resetting);
    if (resetting && !b)
      enable();
    else if (!resetting && b)
      reset();
  }

  public void init() {
    m_simcontrol.init();
  }

  public void run() {
    m_simcontrol.run();
  }
}
