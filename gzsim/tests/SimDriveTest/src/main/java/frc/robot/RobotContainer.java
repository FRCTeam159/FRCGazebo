// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DrivePath;
import frc.robot.commands.Calibrate;
import frc.robot.commands.DriveWithGamepad;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Trajectories;
import utils.PlotUtils;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final XboxController m_controller = new XboxController(0);

  private DriveWithGamepad m_driveCommand = null; // TODO
  public static boolean calibrate = false;

  SendableChooser<Command> m_chooser = new SendableChooser<>();
  SendableChooser<Integer> m_plot_options = new SendableChooser<>();

  Calibrate m_calibrate=new Calibrate(m_drivetrain);
  DrivePath m_straightpath = new DrivePath(m_drivetrain,Trajectories.STRAIGHT);
  DrivePath m_curvedpath = new DrivePath(m_drivetrain,Trajectories.CURVED);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    m_driveCommand=new DriveWithGamepad(m_drivetrain, m_controller);
    m_drivetrain.setDefaultCommand(m_driveCommand);

    m_chooser.setDefaultOption("Curved Path", m_curvedpath);
    m_chooser.addOption("Straight Path", m_straightpath);
    m_chooser.addOption("Calibrate", m_calibrate);

    m_plot_options.setDefaultOption("No Plot", PlotUtils.PLOT_NONE);
    m_plot_options.addOption("Plot Path", PlotUtils.PLOT_PATH);
    m_plot_options.addOption("Plot Dynamics", PlotUtils.PLOT_DYNAMICS);

    SmartDashboard.putData(m_chooser);
    SmartDashboard.putData(m_plot_options);

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
      PlotUtils.plot_option=m_plot_options.getSelected();
      return m_chooser.getSelected();
  }
  public void robotInit(){
    m_drivetrain.simulation.init();
  }
  public void simulationInit(){
    System.out.println("simulationInit()");
    m_drivetrain.enable();
    m_drivetrain.simulation.run();  
  }
}
