// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gazebo.SimClock;
import gazebo.SimControl;

public class Simulation extends SubsystemBase {
  /** Creates a new SimulationControl. */
  private SimControl m_simcontrol = new SimControl();
  private Drivetrain m_drivetrain;
  private boolean resetting = false;

  double simtime=0;

  private SimClock m_simclock = new SimClock();

  public Simulation(Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    SmartDashboard.putBoolean("Reset", false);
    SmartDashboard.putNumber("SimTime", 0);
    SmartDashboard.putNumber("SimClock", 0);
  }

  public double getTime(){
    return m_simclock.getTime();
  }
  public void reset() {
    m_drivetrain.reset(); // reset encoders
    m_simclock.reset();
    resetting=true;
    SmartDashboard.putNumber("SimTime", 0);
  }

  public void start() {
    SmartDashboard.putBoolean("Reset", false);
    m_simclock.reset();
    m_simclock.enable();
    m_drivetrain.enable();
  }

  public void enable() {
    m_drivetrain.enable();
    SmartDashboard.putBoolean("Reset", false);
    resetting=false;
  }
  public void end() {
     SmartDashboard.putNumber("SimTime", m_simclock.getTime());
     m_simclock.disable();
   }
  @Override
  public void periodic() {
    //SmartDashboard.putNumber("SimTime", m_simclock.getTime());
    // This method will be called once per scheduler run
  }

  public void simulationPeriodic() {
    simtime=getTime();
    SmartDashboard.putNumber("SimClock", getTime());
    boolean b = SmartDashboard.getBoolean("Reset", false);
    //if(b )
    //  reset();
    //SmartDashboard.putBoolean("Reset", false);
    
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
