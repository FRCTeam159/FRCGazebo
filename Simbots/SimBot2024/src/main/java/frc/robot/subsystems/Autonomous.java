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
import frc.robot.commands.TurnToAngle;
import utils.PlotUtils;

public class Autonomous extends SequentialCommandGroup  {
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  SendableChooser<Integer> m_auto_plot_option = new SendableChooser<>();

  Drivetrain m_drive;

  public static final int CALIBRATE = 0;
  public static final int PROGRAM = 1;
  public static final int AUTOTEST = 2;
  public static final int PATHWEAVER = 3;
  public static final int PATHPLANNER = 4;

  boolean reversed=false;
 
  public int selected_path=PROGRAM;

  public static boolean debug_commands=false;

  SendableChooser<Integer> m_path_chooser = new SendableChooser<Integer>();
  /** Creates a new AutoCommands. */
  public Autonomous(Drivetrain drive) {
    m_drive=drive;
   
    m_auto_plot_option.setDefaultOption("No Plot", PlotUtils.PLOT_NONE);
    m_auto_plot_option.addOption("Plot Dynamics", PlotUtils.PLOT_DYNAMICS);
    m_auto_plot_option.addOption("Plot Location", PlotUtils.PLOT_LOCATION);
    m_auto_plot_option.addOption("Plot Position", PlotUtils.PLOT_POSITION);

    m_path_chooser.setDefaultOption("Program", PROGRAM);
	  m_path_chooser.addOption("AutoTest", AUTOTEST);
    m_path_chooser.addOption("PathPlanner", PATHPLANNER);
    m_path_chooser.addOption("Calibrate", CALIBRATE);

    //SmartDashboard.putBoolean("reversed", reversed);
    //SmartDashboard.putBoolean("debug", false);

    SmartDashboard.putNumber("xPath", 5);
    SmartDashboard.putNumber("yPath", -3);
    SmartDashboard.putNumber("rPath", 30.0);
   
		SmartDashboard.putData(m_path_chooser);
    SmartDashboard.putData(m_auto_plot_option);
  }
  public SequentialCommandGroup  getCommand(){
    PlotUtils.auto_plot_option=m_auto_plot_option.getSelected();
    selected_path=m_path_chooser.getSelected();
    //reversed = SmartDashboard.getBoolean("reversed", reversed);
    
    //clearGroupedCommands();
      
    switch (selected_path){
    case CALIBRATE:
      return new SequentialCommandGroup(new Calibrate(m_drive));
    case PROGRAM:
      return new SequentialCommandGroup(new DrivePath(m_drive,PROGRAM,reversed));
    case PATHPLANNER:
      return new SequentialCommandGroup(new DrivePath(m_drive,PATHPLANNER,reversed));
   case AUTOTEST:
     //return new SequentialCommandGroup(new TurnToAngle(m_drive,45.0));
      return new SequentialCommandGroup(
        new DriveBack(m_drive,1.0)
       // new TurnToAngle(m_drive,90.0)
       //, new DrivePath(m_drive,PROGRAM,reversed)
        );
    }
    return null;
  }
}
