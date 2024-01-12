// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveWithGamepad;
import objects.PlotServer;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Autonomous m_autonomous = new Autonomous(m_drivetrain);
  private final XboxController m_controller = new XboxController(0);
  PlotServer m_plotsub=new PlotServer();


  private DriveWithGamepad m_driveCommand = null; // TODO
 
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_driveCommand=new DriveWithGamepad(m_drivetrain, m_controller);
    m_drivetrain.setDefaultCommand(m_driveCommand);
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autonomous.getCommand();
  }
  public void teleopInit(){
    m_drivetrain.setRobotDisabled(false);
    m_drivetrain.setFieldOriented(m_drivetrain.isGyroEnabled());
  }
  public void autonomousInit(){
    m_drivetrain.setRobotDisabled(false);
    m_drivetrain.setFieldOriented(true);
  }
  public void disabledInit(){
    m_drivetrain.setRobotDisabled(true);
    m_drivetrain.disable();
  }
  public void robotInit(){
    m_drivetrain.setRobotDisabled(true);
    m_drivetrain.init();
    m_plotsub.start();
  }
}
