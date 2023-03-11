// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DriveWithGamepad;
import frc.robot.commands.PassThru;
import frc.robot.commands.PoseDualArm;
import frc.robot.commands.PoseOneArm;
import frc.robot.objects.PlotServer;
import frc.robot.subsystems.TagDetector;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OneArm;

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
  private final TagDetector m_detector= new TagDetector(m_drivetrain);
  private final Claw m_claw=new Claw();
  private final Arm m_arm = new Arm();
  private final Autonomous m_autonomous = new Autonomous(m_drivetrain, m_arm, m_claw);

  private DriveWithGamepad m_driveCommand = null; 

  PlotServer m_plotsub=new PlotServer();

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
  
  public void teleopInit() {
    //CommandScheduler.getInstance().schedule(new PassThru(m_arm, m_controller));
    //CommandScheduler.getInstance().schedule(new PoseTwistArm(m_arm,m_claw,m_controller));
    m_drivetrain.setUseTags(false);

    CommandScheduler.getInstance().schedule(new PoseDualArm(m_arm, m_claw, m_controller));

    //m_drivetrain.setRobotDisabled(false);
    m_drivetrain.setFieldOriented(true);
  }
  public void autonomousInit(){
   // m_drivetrain.setRobotDisabled(false);
    m_drivetrain.setFieldOriented(false);
  }
  public void disabledInit(){
    m_drivetrain.disabledInit();
  }
  public void disabledPeriodic() {   
  }
  public void robotInit(){
    //m_drivetrain.setRobotDisabled(true);
    m_drivetrain.init();
    m_arm.start();
    m_detector.start();
    m_plotsub.start();
  }
}
