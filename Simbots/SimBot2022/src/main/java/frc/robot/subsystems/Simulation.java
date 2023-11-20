// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gazebo.SimClock;
import frc.robot.objects.SimControl;

public class Simulation extends SubsystemBase {

  /** Creates a new SimulationControl. */
  private SimControl m_simcontrol = new SimControl();
  private DriveTrain m_drive;
  private Shooting m_shoot;
  private Climber m_climber;
  private Targeting m_targeting;
  private static boolean resetting = false;
  private final Field2d m_fieldSim = new Field2d();
  
  private static final Timer m_timer = new Timer();

  double simtime = 0;
  private static boolean running = false;
  private static boolean disabling = false;

  private static SimClock m_simclock = new SimClock();

  public Simulation(DriveTrain drivetrain, Shooting shoot,Climber climber, Targeting targeting) {
    m_drive = drivetrain;
    m_shoot = shoot;
    m_climber = climber;
    m_targeting=targeting;
    SmartDashboard.putBoolean("Reset", false);
    SmartDashboard.putBoolean("Gazebo", false);
    SmartDashboard.putNumber("SimTime", 0);
    SmartDashboard.putData("Field", m_fieldSim);
    m_timer.start();
  }

  public static double getSimTime() {
    return m_simclock.getTime();
  }

  public static double getClockTime() {
    return m_timer.get();
  }

  public void reset() {
    //System.out.println("Simulation.reset");
    m_simclock.reset();
    m_timer.reset();
    SmartDashboard.putNumber("SimTime", 0);
   // running = false;
  }
  public void clear() {
    //System.out.println("Simulation.clear");
    m_simclock.clear();
    m_timer.reset();
    m_drive.resetPose();
    SmartDashboard.putNumber("SimTime", 0);
    running = false;
  }

  public static void start() {
    //System.out.println("Simulation.start");
    m_simclock.reset();
    m_simclock.enable();
    running = true;
  }

  public void end() {
    //System.out.println("Simulation.end");
    SmartDashboard.putNumber("SimTime", m_simclock.getTime());
    m_simclock.disable();
    running = false;
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("SimTime", m_simclock.getTime());
    // This method will be called once per scheduler run
  }

  // set the robot pose in the glass field window
  // - field_offset is the initila position of the robot on the field
  // - for 2022 RapidReact needed to scale down effective travel distance
  //   by 10% in order to match robot position in Gazebo simulator
  private void setFieldPose() {
    Pose2d field_offset= new Pose2d(7.3,2,Rotation2d.fromDegrees(90));

    Pose2d field_pose = m_drive.getFieldPose();
    Pose2d p1 = DriveTrain.add(field_offset, field_pose);

    Pose2d robot_pose = m_drive.getPose();
    Translation2d t1 = robot_pose.getTranslation();
    t1 = t1.times(0.9);
    robot_pose = new Pose2d(t1, robot_pose.getRotation());
    Pose2d sim_pose = DriveTrain.add(p1, robot_pose);

    m_fieldSim.setRobotPose(sim_pose);
  }

  public void simulationPeriodic() {
    setFieldPose();
    SmartDashboard.putNumber("SimTime", getSimTime());
    
    boolean m = SmartDashboard.getBoolean("Gazebo", false);
    boolean b = SmartDashboard.getBoolean("Reset", false);
    if (b) {
      if (!resetting) {
        m_shoot.reset();       
        resetting = true;
        if (m)
          clear();       
        m_drive.reset();
        m_climber.reset();
        m_targeting.reset();
        m_timer.reset();
      } else if (m_timer.get() > 0.1) {
        if (!disabling) {
          m_drive.disable();
          end();
          disabling = true;
        } else if (m_timer.get() > 0.5) {
          SmartDashboard.putBoolean("Reset", false);
          resetting = false;
          disabling = false;
          m_drive.enable();
          m_climber.enable();
          run();
          running = true;
        }
      }
    }
  }

  public void init() {
    m_simcontrol.init();
    m_simclock.disable();
    run();
  }

  public void disable() {
    end();
  }

  public void startAuto() {
    reset();
		start();
  }

  public void run() {
    running = true;
    m_simcontrol.run();
  }
}
