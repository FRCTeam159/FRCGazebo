// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gazebo.SimClock;
import frc.robot.objects.SimControl;
import gazebo.SimCamera;

public class Simulation extends SubsystemBase {
  static public boolean debug=true;
  /** Creates a new SimulationControl. */
  private SimControl m_simcontrol = new SimControl();
  private DriveTrain m_drive;
  private boolean resetting = false;

  //private final Field2d m_fieldSim = new Field2d();

  private final Timer m_timer = new Timer();

  double simtime = 0;
  private boolean running = false;
  private boolean disabling = false;

  private SimClock m_simclock = new SimClock();
  private ArrayList<SimCamera> m_cameras;

  public Simulation(DriveTrain drivetrain) {
    m_drive = drivetrain;
    SmartDashboard.putBoolean("Reset", true);
    SmartDashboard.putBoolean("Gazebo", true);
    SmartDashboard.putNumber("SimTime", 0);
    SmartDashboard.putNumber("SimClock", 0);
    //SmartDashboard.putData("Field", m_fieldSim);
    m_timer.start();
    m_simclock.clear();
    m_cameras = new ArrayList<SimCamera>();
  }

  public double getSimTime() {
    if(!running)
    return 0;
    return m_simclock.getTime();
  }

  public double getClockTime() {
    return m_timer.get();
  }

  public void reset() {
    if(debug)
    System.out.println("Simulation.reset");
    m_simclock.reset();
    m_timer.reset();
    SmartDashboard.putNumber("SimTime", 0);
   // running = false;
  }
  public void clear() {
    if(debug)
    System.out.println("Simulation.clear");
    m_simclock.clear();
    m_timer.reset();
    //m_drive.resetPose();
    SmartDashboard.putNumber("SimTime", 0);
    running = false;
  }

  public void addCamera(SimCamera c) {
    m_cameras.add(c);
  }

  public SimCamera getCamera(int n) {
    for (int i = 0; i < m_cameras.size(); i++) {
      SimCamera c = m_cameras.get(i);
      if (c.getChannel() == n && c.isEnabled()) {
        return c;
      }
    }
    return null;
  }

  public void startCamera(int i) {
    SimCamera camera = getCamera(i);
    if (camera != null && !camera.isRecording())
      camera.run();
  }

  public void stopCamera(int i) {
    SimCamera camera = getCamera(i);
    if (camera != null && camera.isRecording())
      camera.stop();
  }

  public void resetCamera(int i) {
    SimCamera camera = getCamera(i);
    if (camera != null)
      camera.reset();
  }

  public void enableCamera(int i) {
    SimCamera camera = getCamera(i);
    if (camera != null && !camera.isEnabled())
      camera.enable();
  }

  public void disableCamera(int i) {
    SimCamera camera = getCamera(i);
    if (camera != null && camera.isEnabled())
      camera.disable();
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
    m_drive.enable();
  }

  public void end() {
    if(debug)
    System.out.println("Simulation.end");
    SmartDashboard.putNumber("SimTime", m_simclock.getTime());
    m_simclock.disable();
    running = false;
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("SimTime", m_simclock.getTime());
    // This method will be called once per scheduler run
  }

  public void simulationPeriodic() {
    simtime = getClockTime();
    if (running)
      SmartDashboard.putNumber("SimClock", getClockTime());
    else
      SmartDashboard.putNumber("SimClock", 0);
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
          m_drive.reset();
          disabling = true;
        } else if (m_timer.get() > 0.5) {
          SmartDashboard.putBoolean("Reset", false);
          resetting = false;
          disabling = false;
          //m_drive.resetPose();
          //m_drive.enable();
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
  }
  public boolean running(){
    return running;
  }
}