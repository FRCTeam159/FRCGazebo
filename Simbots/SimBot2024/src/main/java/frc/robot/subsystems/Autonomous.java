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

  static double angle=-60;
  
  static double d2r=2*Math.PI/360;
  static double i2m=0.0254;

  static double y=100*i2m;
  static double x=5*i2m; // should be 23 ??

  static double cos=Math.cos(d2r*angle);
  static double sin=Math.sin(d2r*angle);
 
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

    SmartDashboard.putNumber("xPath", -1.6);
    SmartDashboard.putNumber("yPath", -2.2);
    SmartDashboard.putNumber("rPath", 60.0);
    
    SmartDashboard.putBoolean("Pathplanner", false);

		SmartDashboard.putData(m_path_chooser);
    SmartDashboard.putData(m_auto_plot_option);
    
  }
  
  public SequentialCommandGroup getCommand() {
    PlotUtils.auto_plot_option = m_auto_plot_option.getSelected();
    selected_path = m_path_chooser.getSelected();

    boolean use_pathplanner=SmartDashboard.getBoolean("Pathplanner", false);

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

        // double xp = x * cos + y * sin;
        // double yp = -x * sin + y * cos;
        double xp = -2.4;
        double yp = -1.5;
        System.out.println("reverse x:" + (yp) + " y:" + xp);

        x = xp * cos - yp * sin;
        y = xp * sin + yp * cos;
        System.out.println("forward x:" + (y) + " y:" + x);
        return new SequentialCommandGroup(
              // new DrivePath(m_drive, opt, -yp, xp, 60),
              // new Pickup(m_drive, 5),
              // new DrivePath(m_drive, opt, y, x, -60)
              new DrivePath(m_drive, opt, -1.5, -2.4, 60),
              new Pickup(m_drive, 2),
              new DrivePath(m_drive, opt, 3.0, -0.3, -60)
        );
      }
    }
    return null;
  }
}
