// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ControlArm;
import frc.robot.commands.DriveWithGamepad;
import frc.robot.commands.Pickup;
import frc.robot.commands.Shoot;
import objects.PlotServer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Drivetrain;
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
  static boolean resetting = false;

  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drive = new Drivetrain();
  private final Arm m_arm = new Arm();
  private final Autonomous m_autonomous = new Autonomous(m_drive, m_arm);
  private final XboxController m_controller = new XboxController(0);
  private final TagDetector m_detector = new TagDetector(m_drive);
  //PlotServer m_plotsub = new PlotServer();

  private DriveWithGamepad m_driveCommand = null; // TODO

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_driveCommand = new DriveWithGamepad(m_drive, m_controller);
    m_drive.setDefaultCommand(m_driveCommand);
    m_arm.setDefaultCommand(new ControlArm(m_arm, m_controller));

    NamedCommands.registerCommand("Pickup", new Pickup(m_arm));
    NamedCommands.registerCommand("Shoot", new Shoot(m_arm));

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autonomous.getCommand();
  }

  public void teleopInit() {
    m_drive.setRobotDisabled(false);
    m_drive.endAuto();
    m_drive.enable();
    Robot.status = "Teleop";
  }

  public void autonomousInit() {
    m_drive.setRobotDisabled(false);
    Autonomous.ok2run = true;
    m_drive.enable();   
    TargetMgr.reset();
    Robot.status = "Autonomous";
  }

  public void disabledInit() {
    m_drive.setRobotDisabled(true);
    m_drive.endAuto();
    Robot.status = "Disabled";
  }

  public void robotInit() {
    m_drive.setRobotDisabled(true);
    m_drive.init();
    //m_plotsub.start();
    m_detector.start();
  }

  public void reset() {
    System.out.println("Robot reset");
    m_drive.reset();
  }

  public void simulationPeriodic() {

    boolean b = SmartDashboard.getBoolean("Reset", false);
    if (b && !resetting) {
      resetting = true;
      m_drive.resetWheels(true);
      m_arm.reset();
      TargetMgr.reset();
    } else if (!b && resetting) {
      resetting = false;
      m_drive.resetPose();
    } else if (resetting && !m_drive.wheelsReset()) {
      m_drive.resetWheels(false);
    }
  }
}
