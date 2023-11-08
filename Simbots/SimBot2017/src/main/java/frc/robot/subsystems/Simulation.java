// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gazebo.SimClock;
import frc.robot.Robot;
import frc.robot.objects.SimControl;
import gazebo.SimCamera;

public class Simulation extends SubsystemBase {
  static public boolean debug=true;
  /** Creates a new SimulationControl. */
  private SimControl m_simcontrol = new SimControl();
  //private DriveTrain m_drive;
  private boolean resetting = false;
  boolean auto_running=false;
  double auto_start_time=0;

  //private final Field2d m_fieldSim = new Field2d();

  private final Timer m_timer = new Timer();
  private boolean running = false;
  private boolean disabling = false;

  private SimClock m_simclock = new SimClock();
  public Simulation() {
    //m_drive = Robot.driveTrain;
    SmartDashboard.putBoolean("Reset", true);
    SmartDashboard.putBoolean("Gazebo", true);
    SmartDashboard.putNumber("AutoTime", 0);
    SmartDashboard.putNumber("SimClock", 0);
    //SmartDashboard.putData("Field", m_fieldSim);
    m_timer.start();
    m_simclock.clear();
    new ArrayList<SimCamera>();
  }

  public double getSimTime() {
    if(!running)
      return 0;
    return m_simclock.getTime();
  }

  public void startAuto(){
    auto_start_time=getClockTime();
    auto_running=true;
  }
  public void endAuto(){
    auto_running=false;
  }
  public double getClockTime() {
    //return m_timer.get();
    return getSimTime();
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
    //m_drive.resetPose();
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
    Robot.driveTrain.enable();
  }

  public void end() {
    if(debug)
      System.out.println("Simulation.end");
    m_simclock.disable();
    running = false;
  }

  public void simulationPeriodic() {
    if (running)
      SmartDashboard.putNumber("SimClock", getClockTime());
    else
      SmartDashboard.putNumber("SimClock", 0);
    if(auto_running)
      SmartDashboard.putNumber("AutoTime", getClockTime()-auto_start_time);
    
    boolean m = SmartDashboard.getBoolean("Gazebo", false);
    boolean b = SmartDashboard.getBoolean("Reset", false);
    if (b) {
      if (!resetting) {
        resetting = true;
        if (m)
          clear();
        m_timer.reset();
      } else if (m_timer.get() > 0.25) {
        if (!disabling) {
          Robot.reset();
          disabling = true;
        } else if (m_timer.get() > 0.5) {
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
