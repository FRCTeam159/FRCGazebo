// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ClimberCommands;
import frc.robot.commands.DriveWithGamepad;
import frc.robot.commands.ShootingCommand;
import frc.robot.objects.CameraStreams;
import frc.robot.subsystems.Targeting;
//import utils.PlotServer;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Cameras;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooting;
import frc.robot.subsystems.Simulation;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final XboxController m_controller = new XboxController(0);
  
  // The robot's subsystems and commands are defined here...
  private final DriveTrain m_drivetrain = new DriveTrain();
  private final Cameras m_cameras = new Cameras();
  private final Targeting m_targeting = new Targeting(m_drivetrain);
  private final CameraStreams m_streams = new CameraStreams(m_targeting);

  private final Shooting m_shoot = new Shooting();
  private final Climber m_climber=new Climber();
  private Simulation m_simulation;
  //private utils.PlotServer m_plotter=new PlotServer();
 
  private final Autonomous m_autonomous = new Autonomous(m_drivetrain,m_targeting,m_shoot);


  private DriveWithGamepad m_driveCommand = null; // TODO
 
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_driveCommand=new DriveWithGamepad(m_drivetrain, m_controller);
    m_drivetrain.setDefaultCommand(m_driveCommand);
    m_simulation=new Simulation(m_drivetrain,m_shoot,m_climber,m_targeting);
    m_shoot.setDefaultCommand(new ShootingCommand(m_shoot, m_controller, m_targeting, m_drivetrain));
    m_climber.setDefaultCommand(new ClimberCommands(m_controller, m_climber));
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
    m_drivetrain.enable();
    m_targeting.reset();
  }
  public void disabledInit(){
    m_targeting.reset();
    m_drivetrain.disable();
    m_simulation.disable();
  }

  public void autonomousInit(){
    //m_drivetrain.disable();
    m_simulation.startAuto();
  }
  
  public void robotInit(){
    m_targeting.reset();
    m_drivetrain.init();
    m_climber.init();
    m_simulation.init();
    m_streams.start();
    // if(Robot.isSimulation()){
    //   m_plotter.start();
    // }
  }
}
