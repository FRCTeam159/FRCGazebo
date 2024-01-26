// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ControlArm;
import frc.robot.commands.DriveWithGamepad;
import objects.PlotServer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.TagDetector;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  static boolean resetting=false;

  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Arm m_arm = new Arm();
  private final Autonomous m_autonomous = new Autonomous(m_drivetrain,m_arm);
  private final XboxController m_controller = new XboxController(0);
  private final TagDetector m_detector= new TagDetector(m_drivetrain);
  PlotServer m_plotsub=new PlotServer();

  private DriveWithGamepad m_driveCommand = null; // TODO
 
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_driveCommand=new DriveWithGamepad(m_drivetrain, m_controller);
    m_drivetrain.setDefaultCommand(m_driveCommand);
    m_arm.setDefaultCommand(new ControlArm(m_arm,m_controller));
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
   // m_drivetrain.setFieldOriented(m_drivetrain.isGyroEnabled());
    m_drivetrain.endAuto();
  }
  public void autonomousInit(){
    m_drivetrain.setRobotDisabled(false);
    //m_drivetrain.setFieldOriented(false);
    m_drivetrain.startAuto();
  }
  public void disabledInit(){
    m_drivetrain.setRobotDisabled(true);
    m_drivetrain.disable();
    m_drivetrain.endAuto();
  }
  public void robotInit(){
    m_drivetrain.setRobotDisabled(true);
    m_drivetrain.init();
    m_plotsub.start();
    m_detector.start();
  }
  public void reset(){
    System.out.println("Robot reset");
    m_drivetrain.reset();
  }
  public void simulationPeriodic(){
    
  boolean b = SmartDashboard.getBoolean("Reset", false);
  if(b &&  !resetting){
    resetting=true;
    m_drivetrain.resetWheels();
    m_arm.reset();
  }
  else if(!b && resetting){
    resetting=false;
    m_drivetrain.resetPose();
  }
  else if(resetting && ! m_drivetrain.wheelsReset()){
    m_drivetrain.allignWheels();
  }
}
}

