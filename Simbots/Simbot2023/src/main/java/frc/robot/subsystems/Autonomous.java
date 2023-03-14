// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Calibrate;
import frc.robot.commands.DriveBack;
import frc.robot.commands.DrivePath;
import frc.robot.commands.PlaceCube;
import frc.robot.commands.TurnTest;
import utils.PlotUtils;

public class Autonomous extends SequentialCommandGroup  {
  SendableChooser<Integer> m_auto_plot_option = new SendableChooser<Integer>();
  SendableChooser<Integer> m_path_chooser = new SendableChooser<Integer>();
  SendableChooser<Integer> m_auto_level_option = new SendableChooser<Integer>();

  Drivetrain m_drive;
  Arm m_arm;
  Claw m_claw;

  public static final int CALIBRATE = 0;
  public static final int PROGRAM = 1;
  public static final int AUTOTEST = 2;
  public static final int PATHWEAVER = 3;
  public static final int PATHPLANNER = 4;

  public int selected_path=PROGRAM;
  public int selected_level=2;

  public static boolean debug_commands=false;

  /** Creates a new AutoCommands. 
   * @param m_claw*/
  public Autonomous(Drivetrain drive,Arm arm, Claw claw) {
    m_drive=drive;
    m_arm=arm;
    m_claw=claw;
   
    m_auto_plot_option.setDefaultOption("No Plot", PlotUtils.PLOT_NONE);
    m_auto_plot_option.addOption("Plot Dynamics", PlotUtils.PLOT_DYNAMICS);
    m_auto_plot_option.addOption("Plot Location", PlotUtils.PLOT_LOCATION);
    m_auto_plot_option.addOption("Plot Position", PlotUtils.PLOT_POSITION);

    m_path_chooser.setDefaultOption("Program", PROGRAM);
	  m_path_chooser.addOption("AutoTest", AUTOTEST);
    m_path_chooser.addOption("PathPlanner", PATHPLANNER);
    m_path_chooser.addOption("Calibrate", CALIBRATE);

    m_auto_level_option.addOption("Level 0", 0);
	  m_auto_level_option.addOption("Level 1", 1);
    m_auto_level_option.setDefaultOption("Level 2", 2);

    SmartDashboard.putNumber("xPath", -2);
    SmartDashboard.putNumber("yPath", 0);
    SmartDashboard.putNumber("rPath", 0);
   
		SmartDashboard.putData(m_path_chooser);
    SmartDashboard.putData(m_auto_plot_option);
    SmartDashboard.putData(m_auto_level_option);
  }
  public SequentialCommandGroup  getCommand(){
    PlotUtils.auto_plot_option=m_auto_plot_option.getSelected();
    selected_path=m_path_chooser.getSelected();
    selected_level=m_auto_level_option.getSelected();
    
    switch (selected_path){
    case CALIBRATE:
      return new SequentialCommandGroup(new Calibrate(m_drive));
    case PROGRAM:
      return new SequentialCommandGroup(new DrivePath(m_drive,PROGRAM));
    case PATHPLANNER:
      return new SequentialCommandGroup(new DrivePath(m_drive,PATHPLANNER));
   case AUTOTEST:
      return new SequentialCommandGroup(
        new PlaceCube(selected_level,m_arm,m_claw)
        ,new DriveBack(m_drive)
        //,new DrivePath(m_drive,PATHPLANNER)
        );
    }
    return null;
  }
}
