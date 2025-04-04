// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ClimberControls;
import frc.robot.commands.ControlArm;
import frc.robot.commands.DriveWithGamepad;
import frc.robot.commands.Pickup;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.DualCameras;
import frc.robot.subsystems.NoteDetector;
import frc.robot.subsystems.TagDetector;
import frc.robot.subsystems.TargetMgr;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  static boolean m_resetting = false;

  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drive = new Drivetrain();
  private final Arm m_arm = new Arm();
  private final Autonomous m_autonomous = new Autonomous(m_drive, m_arm);
  private final XboxController m_controller = new XboxController(0);
  private final TagDetector m_tag_detector = new TagDetector();
  private final NoteDetector m_note_detector = new NoteDetector();
  private final DualCameras m_cameras=new DualCameras();
  private Climber m_climber = new Climber();

  
  private DriveWithGamepad m_driveCommand = null; 

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_driveCommand = new DriveWithGamepad(m_drive, m_controller);
    m_drive.setDefaultCommand(m_driveCommand);
    m_arm.setDefaultCommand(new ControlArm(m_arm, m_drive, m_controller));
    m_climber.setDefaultCommand(new ClimberControls(m_climber, m_controller));

    NamedCommands.registerCommand("Pickup", new Pickup(m_arm));
    NamedCommands.registerCommand("Shoot", new Shoot(m_arm,m_drive));
  }

  public Command getAutonomousCommand() {
    return m_autonomous.getCommand();
  }

  public void robotInit() {
    Robot.disabled=true;
    m_drive.init();
    m_cameras.start();
    Robot.status = "Init";
    Autonomous.end(); 
  }

  
  public void teleopInit() {
    Robot.disabled=false;
    m_drive.endAuto();
    m_drive.enable();
    Autonomous.end(); 
    Robot.status = "Teleop";
  }

  public void autonomousInit() {
    Robot.disabled=false;
    m_drive.enable();
    Autonomous.start(); 
    TargetMgr.reset();
    Robot.status = "Autonomous";
  }

  public void disabledInit() {
    Robot.disabled=true;
    m_drive.endAuto();
    Robot.status = "Disabled";
    Autonomous.end(); 
  }
  
  public void reset() {
    System.out.println("Robot reset");
    m_drive.reset();
  }

  public void simulationPeriodic() {
    boolean b = SmartDashboard.getBoolean("Reset", false);
    if (b && !m_resetting) { // start aligning
      m_resetting = true;
      m_drive.resetWheels(true);
      m_arm.reset();
      m_climber.reset();
      Robot.initialized=false;
      TargetMgr.reset();
      TagDetector.setTargeting(false);
    } else if (!b && m_resetting) { // end aligning
      m_resetting = false;
      m_drive.resetPose();
    } else if (m_resetting && !m_drive.wheelsReset()) { // align the wheels
      m_drive.resetWheels(false);
    }
  }
}
