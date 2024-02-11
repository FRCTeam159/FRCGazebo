// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
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
  public static boolean goback=true; // return to zero from current pose
 

  //public int selected_path=PROGRAM;

  public static boolean debug_commands=false;
  public static boolean ok2run=false;
  static boolean m_reversed=false;
  static boolean m_autoselect=true;
  static boolean m_usetags=false;
  static boolean m_use_pathplanner=true;

  /** Creates a new AutoCommands. */
  public Autonomous(Drivetrain drive,Arm arm) {
    m_drive=drive;
    m_arm=arm;
    SmartDashboard.putNumber("xPath", xp);
    SmartDashboard.putNumber("yPath", yp);
    SmartDashboard.putNumber("rPath", rp);

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

    SmartDashboard.putBoolean("reversed",m_reversed);
    SmartDashboard.putBoolean("Autoset",m_autoselect);
    SmartDashboard.putBoolean("UseTags",m_usetags);

    SmartDashboard.putBoolean("Pathplanner", m_use_pathplanner);
    
    m_auto_plot_option.setDefaultOption("No Plot", PlotUtils.PLOT_NONE);
    m_auto_plot_option.addOption("Plot Dynamics", PlotUtils.PLOT_DYNAMICS);
    m_auto_plot_option.addOption("Plot Location", PlotUtils.PLOT_LOCATION);
    m_auto_plot_option.addOption("Plot Position", PlotUtils.PLOT_POSITION);
    SmartDashboard.putData(m_auto_plot_option);

  }
  static public int getAlliance(){
    return m_alliance_chooser.getSelected();
  }
  static public int getPosition(){
    return m_position_chooser.getSelected();
  }
  static public boolean getReversed(){
    return SmartDashboard.getBoolean("reversed",m_reversed);
  }
  static public boolean getAutoset(){
    return SmartDashboard.getBoolean("Autoset",m_autoselect);
  }
  static public boolean getUsetags(){
    return SmartDashboard.getBoolean("UseTags",m_usetags);
  }
  static public boolean getUsePathplanner(){
    return SmartDashboard.getBoolean("Pathplanner",m_use_pathplanner);
  }
  public SequentialCommandGroup getCommand(){
    return new SequentialCommandGroup(new InitAuto(m_arm),getAutoSequence());
  }
  SequentialCommandGroup getAutoSequence() {
    PlotUtils.auto_plot_option = m_auto_plot_option.getSelected();
    int selected_path = m_path_chooser.getSelected();

    boolean use_pathplanner=getUsePathplanner();
    int opt=use_pathplanner?PROGRAMPP:PROGRAM;      

    switch (selected_path) {
      case CALIBRATE:
        return new SequentialCommandGroup(new Calibrate(m_drive));
      case PROGRAM:
        return new SequentialCommandGroup(
          new AlignWheels(m_drive,2),
          new DrivePath(m_drive, opt)
        );
      case PATHPLANNER:
      {
        int position=TargetMgr.getStartPosition();
        String pathname="RightSideZeroed";
        if(position==TargetMgr.CENTER)
          pathname="CenterZeroed";
        
        Pose2d p = PathPlannerAuto.getStaringPoseFromAutoFile(pathname);
        if(p!=null)
          m_drive.resetOdometry(p);
        Command cmd;
        if(use_pathplanner){
          List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(pathname);
          return new SequentialCommandGroup(
            new Shoot(m_drive,m_arm),
            AutoBuilder.followPath(pathGroup.get(0)),
            new Pickup(m_arm, m_drive,3),
            AutoBuilder.followPath(pathGroup.get(1)),
            new Shoot(m_drive,m_arm)
          );
        }
        else{
          cmd=AutoBuilder.buildAuto(pathname);   
          return new SequentialCommandGroup(cmd);
        }     
      }
      case AUTOTEST: {   
        return new SequentialCommandGroup(
              new AlignWheels(m_drive,2),
              new Shoot(m_drive,m_arm),
              new DrivePath(m_drive, opt, false),
              goback?new Pickup(m_arm, m_drive,3):
                new ParallelCommandGroup(       
                new Pickup(m_arm, null,3),new AlignWheels(m_drive,2)),
              new DrivePath(m_drive, opt, true),
              new Shoot(m_drive,m_arm)
        );
      }
    }
    return null;
  }



  
}
