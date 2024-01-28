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
import frc.robot.commands.Shoot;
import utils.PlotUtils;

public class Autonomous extends SequentialCommandGroup  {
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  SendableChooser<Integer> m_auto_plot_option = new SendableChooser<>();

  Drivetrain m_drive;
  Arm m_arm;

  public static final int CALIBRATE = 0;
  public static final int PROGRAM = 1;
  public static final int PROGRAMPP = 2;
  public static final int PATHPLANNER = 3;
  public static final int AUTOTEST = 4;
  
  static double d2r=2*Math.PI/360;
  static double i2m=0.0254;

  
  static double XF=0.9;
  static double YF=-1.3;
  static double RF=-60;

  static double YR=-0.5;
  static double XR=-1.3;
  static double RR=60;

  static double xp=XF;
  static double yp=YF;
  static double rp=RF;

  static boolean test_coord_rotation=false;

  public int selected_path=PROGRAM;

  public static boolean debug_commands=false;

  SendableChooser<Integer> m_path_chooser = new SendableChooser<Integer>();
  /** Creates a new AutoCommands. */
  public Autonomous(Drivetrain drive,Arm arm) {
    m_drive=drive;
    m_arm=arm;
   
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

    // TODO
    // 1) calulate forward & reverse paths based on starting position and alliance
    // 2) use field geometry of robot starting position and closest note to constuct paths
    // 3) use pathplanner paths ?
  
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
        int alliance=TargetMgr.alliance;
        int position=TargetMgr.position;
        // set up default for center case
        double xr=XR;
        double yr=0;
        double rr=0;
        double xf=-XR;
        double yf=0;
        double rf=0;
        System.out.println(TargetMgr.getStartString());
        //for a simple curved path blue-outside=red-inside and blue-inside=red-outside
        if((alliance==TargetMgr.RED && position==TargetMgr.OUTSIDE) ||
           (alliance==TargetMgr.BLUE && position==TargetMgr.INSIDE)){
            xf=XF;
            yf=-YF;
            rf=-RF;
            xr=XR;
            yr=-YR;
            rr=-RR;

        }
        if((alliance==TargetMgr.BLUE && position==TargetMgr.OUTSIDE) ||
          (alliance==TargetMgr.RED && position==TargetMgr.INSIDE)){
            xf=XF;
            yf=YF;
            rf=RF;
            xr=XR;
            yr=YR;
            rr=RR;
        }
        //rf=-rr;
        if(test_coord_rotation){
          double xrot=xr;
          double yrot=yr;
          double rrot=rr;
          double cos=Math.cos(d2r*rrot);
          double sin=Math.sin(d2r*rrot);
          double x = yrot * cos + xrot * sin;
          double y = -yrot * sin + xrot * cos;
          System.out.println("reverse x:" + xr + " y:" + yr + " r:"+rr);
          System.out.println("forward x:" + (x) + " y:" + y+ " r:"+rf);
        }
      
        return new SequentialCommandGroup(
              new Shoot(m_drive,m_arm),
              new DrivePath(m_drive, opt, xf, yf, rf),
              new Pickup(m_drive, m_arm, 4),
              new DrivePath(m_drive, opt, xr,yr,rr),
              new Shoot(m_drive,m_arm)
        );
      }
    }
    return null;
  }
}
