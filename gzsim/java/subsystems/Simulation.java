// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gazebo.SimClock;
import objects.SimControl;

public class Simulation extends SubsystemBase {
  static public boolean debug=true;
  /** Creates a new SimulationControl. */
  private SimControl m_simcontrol = new SimControl();
  boolean auto_running=false;
  double auto_start_time=0;

  private final Timer m_timer = new Timer();

  public static boolean running = false;
  public static boolean resetting = false;
  public static boolean disabling = false;

  private SimClock m_simclock = new SimClock();
  
  public Simulation() {
    SmartDashboard.putBoolean("Reset", true);
    SmartDashboard.putBoolean("Gazebo", true);
    SmartDashboard.putNumber("AutoTime", 0);
    SmartDashboard.putNumber("SimClock", 0);
    //SmartDashboard.putData("Field", m_fieldSim);
    m_timer.start();
    m_simclock.clear();
  }

  public double getSimTime() {
      return m_simclock.getTime();
  }

  public void startAuto(){
    auto_start_time=getSimTime();
    auto_running=true;
  }
  public void endAuto(){
    auto_running=false;
  }
  public double getClockTime() {
    return m_timer.get();
  }

  public double getAutoTime() {
    return getSimTime()-auto_start_time;
  }

  public void reset() {
    if(debug)
      System.out.println("Simulation.reset");
    m_simclock.reset();
    m_timer.reset();
  }
  public void clear() {
    if(debug)
      System.out.println("Simulation.clear");
    m_simclock.clear();
    m_timer.reset();
    running = false;
  }

  public void start() {
    if(debug)
      System.out.println("Simulation.start");
    m_simclock.reset();
    m_simclock.enable();
    running = true;
  }

  public void enable() {
    if(debug)
      System.out.println("Simulation.enable");
  }

  public void end() {
    if(debug)
      System.out.println("Simulation.end");
    m_simclock.disable();
    running = false;
  }

  public void simulationPeriodic() {
    if (running)
      SmartDashboard.putNumber("SimClock", getSimTime());
    else
      SmartDashboard.putNumber("SimClock", 0);
    if(auto_running)
      SmartDashboard.putNumber("AutoTime", getAutoTime());
    
    boolean m = SmartDashboard.getBoolean("Gazebo", false);
    boolean b = SmartDashboard.getBoolean("Reset", false);
    if (b) {
      if (!resetting) {
        resetting = true;
        if (m)
          clear();
        m_timer.reset();
      } else if (m_timer.get() > 0.5) {
        if (!disabling) {
          disabling = true;
        } else if (m_timer.get() > 1) {
          SmartDashboard.putBoolean("Reset", false);
          resetting = false;
          disabling = false;
          run();
          running = true;
        }
      }
    }
  }

  public void disable() {
    end();
  }
  
  public boolean started(){
    return m_simclock.started();

  }
  public void init() {
    m_simcontrol.init();
    m_simclock.disable();
  }

  public void run() {
    running = true;
    m_simcontrol.run();
    start();
  }
  public boolean running(){
    return running;
  }
}
