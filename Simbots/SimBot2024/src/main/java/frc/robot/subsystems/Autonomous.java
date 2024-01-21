// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Calibrate;
import frc.robot.commands.DrivePath;
import frc.robot.commands.Pickup;
import utils.PlotUtils;

public class Autonomous extends SequentialCommandGroup  {
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  SendableChooser<Integer> m_auto_plot_option = new SendableChooser<>();

  Drivetrain m_drive;

  public static final int CALIBRATE = 0;
  public static final int PROGRAM = 1;
  public static final int PROGRAMPP = 2;
  public static final int PATHPLANNER = 3;
  public static final int AUTOTEST = 4;
  
  static double d2r=2*Math.PI/360;
  static double i2m=0.0254;

  // blue outside path
  static double xp=-1.0;
  static double yp=1.3;
  static double rp=-60;

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
	  m_path_chooser.addOption("Path", PATHPLANNER);
    m_path_chooser.addOption("AutoTest", AUTOTEST);
    m_path_chooser.addOption("Calibrate", CALIBRATE);

    SmartDashboard.putNumber("xPath", xp);
    SmartDashboard.putNumber("yPath", yp);
    SmartDashboard.putNumber("rPath", rp);
    SmartDashboard.putBoolean("Pathplanner", false);

		SmartDashboard.putData(m_path_chooser);
    SmartDashboard.putData(m_auto_plot_option);
    
  }
  
  public SequentialCommandGroup getCommand() {
    PlotUtils.auto_plot_option = m_auto_plot_option.getSelected();
    selected_path = m_path_chooser.getSelected();

    boolean use_pathplanner=SmartDashboard.getBoolean("Pathplanner", false);

    System.out.println("reverse x:" + (xp) + " y:" + yp);

    double cos=Math.cos(d2r*rp);
    double sin=Math.sin(d2r*rp);
    double x = yp * cos + xp * sin;
    double y = -yp * sin + xp * cos;
    System.out.println("forward x:" + (x) + " y:" + y);

    switch (selected_path) {
      case CALIBRATE:
        return new SequentialCommandGroup(new Calibrate(m_drive));
      case PROGRAM:
      if(!use_pathplanner)
        return new SequentialCommandGroup(new DrivePath(m_drive, PROGRAM));
      else
        return new SequentialCommandGroup(new DrivePath(m_drive, PROGRAMPP));
      case PATHPLANNER:
        return new SequentialCommandGroup(new DrivePath(m_drive, PATHPLANNER));
      case AUTOTEST: {
        int opt=use_pathplanner?PROGRAMPP:PROGRAM;
        System.out.println("pathplanner=" + use_pathplanner);
        return new SequentialCommandGroup(
              new DrivePath(m_drive, opt, xp, yp, rp),
              new Pickup(m_drive, 2),
              new DrivePath(m_drive, opt, x, y,-rp)
        );
      }
    }
    return null;
  }
}
