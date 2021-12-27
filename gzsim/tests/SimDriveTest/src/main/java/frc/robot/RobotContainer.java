// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DrivePath;
import frc.robot.commands.ControlSimulation;
import frc.robot.commands.DefaultAuto;
import frc.robot.commands.DriveWithGamepad;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Simulation;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Simulation m_simulation = new Simulation(m_drivetrain);
  private final XboxController m_controller = new XboxController(0);

  private DrivePath m_autoCommand = null; // TODO
  private ControlSimulation m_simCommand = null; // TODO
  private DriveWithGamepad m_driveCommand = null; // TODO

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    m_driveCommand=new DriveWithGamepad(m_drivetrain, m_controller);
    m_drivetrain.setDefaultCommand(m_driveCommand);
    //m_simCommand=new ControlSimulation(m_simulation);
   // m_simulation.setDefaultCommand(m_simCommand);
    //m_autoCommand=new DefaultAuto(m_drivetrain);
    m_autoCommand=new DrivePath(m_drivetrain);

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
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
  public void robotInit(){
    m_simulation.init();
    //SmartDashboard.putData(m_simCommand);
  }
  public void simulationInit(){
    
    System.out.println("simulationInit()");
    m_drivetrain.enable();
    m_simulation.run();  
  }
}
