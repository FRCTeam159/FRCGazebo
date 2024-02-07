// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlignWheels;
import frc.robot.commands.Calibrate;
import frc.robot.commands.DrivePath;
import frc.robot.commands.InitAuto;
import frc.robot.commands.Pickup;
import frc.robot.commands.Shoot;
import utils.PlotUtils;

public class Autonomous extends SequentialCommandGroup  {
  static SendableChooser<Integer> m_path_chooser = new SendableChooser<Integer>();
  static SendableChooser<Integer> m_position_chooser = new SendableChooser<Integer>();
  static SendableChooser<Integer> m_alliance_chooser = new SendableChooser<Integer>();
  static SendableChooser<Integer> m_auto_plot_option = new SendableChooser<>();

  Drivetrain m_drive;
  Arm m_arm;

  public static final int CALIBRATE = 0;
  public static final int PROGRAM = 1;
  public static final int PROGRAMPP = 2;
  public static final int PATHPLANNER = 3;
  public static final int AUTOTEST = 4;
  
  static double d2r=2*Math.PI/360;
  static double i2m=0.0254;

  static double xp=TargetMgr.XF;
  static double yp=TargetMgr.YF;
  static double rp=TargetMgr.RF;

  static boolean test_coord_rotation=false;

  public int selected_path=PROGRAM;

  public static boolean debug_commands=false;
  public static boolean ok2run=false;

      /** Creates a new AutoCommands. */
  public Autonomous(Drivetrain drive,Arm arm) {
    m_drive=drive;
    m_arm=arm;
    SmartDashboard.putNumber("xPath", xp);
    SmartDashboard.putNumber("yPath", yp);
    SmartDashboard.putNumber("rPath", rp);

    SmartDashboard.putBoolean("Pathplanner", false);

    m_auto_plot_option.setDefaultOption("No Plot", PlotUtils.PLOT_NONE);
    m_auto_plot_option.addOption("Plot Dynamics", PlotUtils.PLOT_DYNAMICS);
    m_auto_plot_option.addOption("Plot Location", PlotUtils.PLOT_LOCATION);
    m_auto_plot_option.addOption("Plot Position", PlotUtils.PLOT_POSITION);
    SmartDashboard.putData(m_auto_plot_option);

    m_path_chooser.setDefaultOption("Program", PROGRAM);
	  m_path_chooser.addOption("Path", PATHPLANNER);
    m_path_chooser.addOption("AutoTest", AUTOTEST);
    m_path_chooser.addOption("Calibrate", CALIBRATE);

    SmartDashboard.putData(m_path_chooser);
   
    m_alliance_chooser.setDefaultOption("Blue", TargetMgr.BLUE);
    m_alliance_chooser.addOption("Red", TargetMgr.RED);
    SmartDashboard.putData(m_alliance_chooser);

    m_position_chooser.setDefaultOption("Outside", TargetMgr.OUTSIDE);
    m_position_chooser.addOption("Center", TargetMgr.CENTER);
    m_position_chooser.addOption("Inside", TargetMgr.INSIDE);
    
    SmartDashboard.putData(m_position_chooser);
     
  }
  static public int getAlliance(){
    return m_alliance_chooser.getSelected();
  }
  static public int getPosition(){
    return m_position_chooser.getSelected();
  }
  public SequentialCommandGroup getCommand(){
    return new SequentialCommandGroup(new InitAuto(m_arm),getAutoSequence());
  }
  SequentialCommandGroup getAutoSequence() {
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
      case PATHPLANNER:{
        String pathname="BlueInside";
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathname);
        if(path==null){
          System.out.println("Pathplanner couldn't find path "+pathname);
          return null;
        }
        Pose2d p;
        Command cmd;
        if(use_pathplanner){
          p = path.getPreviewStartingHolonomicPose();
          cmd=AutoBuilder.followPath(path);
        }
        else{
          cmd=AutoBuilder.buildAuto(pathname);
          p = PathPlannerAuto.getStaringPoseFromAutoFile(pathname);
        }
       
        if(p!=null)
          m_drive.resetOdometry(p);
        else
          System.out.println("Pathplanner error - starting pose is null !");
       
        return new SequentialCommandGroup(cmd);
      }
      case AUTOTEST: {
        int opt=use_pathplanner?PROGRAMPP:PROGRAM;      
        return new SequentialCommandGroup(
              new AlignWheels(m_drive,2),
              new Shoot(m_drive,m_arm),
              new DrivePath(m_drive, opt, false),
              new ParallelCommandGroup(
                new Pickup(m_arm, 4),new AlignWheels(m_drive,2)),
              new DrivePath(m_drive, opt, true),
              new Shoot(m_drive,m_arm)
        );
      }
    }
    return null;
  }
}
