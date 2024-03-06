// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlignWheels;
import frc.robot.commands.AutoTarget;
import frc.robot.commands.Calibrate;
import frc.robot.commands.DrivePath;
import frc.robot.commands.EndAuto;
import frc.robot.commands.GetStartPose;
import frc.robot.commands.Pickup;
import frc.robot.commands.Shoot;
import frc.robot.commands.StartAuto;
import utils.PlotUtils;

public class Autonomous extends SequentialCommandGroup {
  static SendableChooser<Integer> m_path_chooser = new SendableChooser<Integer>();
  static SendableChooser<Integer> m_position_chooser = new SendableChooser<Integer>();
  static SendableChooser<Integer> m_alliance_chooser = new SendableChooser<Integer>();
  static SendableChooser<Integer> m_auto_plot_option = new SendableChooser<>();

  Drivetrain m_drive;
  Arm m_arm;

  public static final int CALIBRATE = 0;
  public static final int PROGRAM = 1;
  public static final int PROGRAMPP = 2;
  public static final int PATH = 3;
  public static final int ONE_NOTE = 4;
  public static final int TWO_NOTE = 5;
  public static final int THREE_NOTE = 6;
  public static final int FOUR_NOTE = 7;

  static double d2r = 2 * Math.PI / 360;
  static double i2m = 0.0254;

  static double xp = TargetMgr.XF;
  static double yp = TargetMgr.YF;
  static double rp = TargetMgr.RF;

  static boolean test_coord_rotation = false;

  public static boolean debug_commands = true;
  static boolean m_ok2run = false;
  static boolean m_inAuto = false;
  static double last_time = 0;

  static boolean m_autoselect = true;
  static boolean m_usetags = false;
  static boolean m_showtags = false;
  static boolean m_reverse = false;
  static boolean m_pathplanner = false;

  /** Creates a new AutoCommands. */
  public Autonomous(Drivetrain drive, Arm arm) {
    m_drive = drive;
    m_arm = arm;
    SmartDashboard.putNumber("xPath", xp);
    SmartDashboard.putNumber("yPath", yp);
    SmartDashboard.putNumber("rPath", rp);

    m_path_chooser.addOption("Program", PROGRAM);
    //m_path_chooser.addOption("Path", PATH);
    m_path_chooser.addOption("OneNote", ONE_NOTE);
    m_path_chooser.addOption("TwoNote", TWO_NOTE);
    m_path_chooser.addOption("ThreeNote", THREE_NOTE);
    m_path_chooser.setDefaultOption("FourNote", FOUR_NOTE);
    //m_path_chooser.addOption("Calibrate", CALIBRATE);
    SmartDashboard.putData(m_path_chooser);

    m_alliance_chooser.setDefaultOption("Blue", TargetMgr.BLUE);
    m_alliance_chooser.addOption("Red", TargetMgr.RED);
    SmartDashboard.putData(m_alliance_chooser);

    m_position_chooser.addOption("Outside", TargetMgr.OUTSIDE);
    m_position_chooser.setDefaultOption("Center", TargetMgr.CENTER);
    m_position_chooser.addOption("Inside", TargetMgr.INSIDE);
    SmartDashboard.putData(m_position_chooser);

    SmartDashboard.putBoolean("Reverse", m_reverse);
    SmartDashboard.putBoolean("Autoset", m_autoselect);
    SmartDashboard.putBoolean("UseTags", m_usetags);
    SmartDashboard.putBoolean("ShowTags", m_showtags);
    SmartDashboard.putBoolean("OkToRun", okToRun());

    SmartDashboard.putBoolean("Pathplanner", m_pathplanner);

    m_auto_plot_option.setDefaultOption("No Plot", PlotUtils.PLOT_NONE);
    m_auto_plot_option.addOption("Plot Dynamics", PlotUtils.PLOT_DYNAMICS);
    m_auto_plot_option.addOption("Plot Location", PlotUtils.PLOT_LOCATION);
    m_auto_plot_option.addOption("Plot Position", PlotUtils.PLOT_POSITION);
    SmartDashboard.putData(m_auto_plot_option);
  }
  
  public static void start(){
    last_time=0;
    m_ok2run=true;
    m_inAuto=true;
    log("Auto Start");
  }
   public static void end(){
    last_time=0;
    m_inAuto=false;
    m_ok2run=true;
    log("Auto End");
  }
  public static boolean okToRun(){
    if(!m_inAuto)
      return true;
    return m_ok2run;
  }
  public static void stop(){
   log("Auto error - aborting !!");
   m_ok2run=false;
  }
  public static boolean running(){
    return m_inAuto;
  }

  static public int getAlliance() {
    return m_alliance_chooser.getSelected();
  }

  static public int getPosition() {
    return m_position_chooser.getSelected();
  }

  static public int getPlacement() {
   int pos=m_position_chooser.getSelected();
   int side=m_alliance_chooser.getSelected();
   return TargetMgr.getPlacement(side,pos);
  }

  static public boolean getReverse() {
    return SmartDashboard.getBoolean("Reverse", m_reverse);
  }

  static public boolean getAutoset() {
    return SmartDashboard.getBoolean("Autoset", m_autoselect);
  }

  static public boolean getUsetags() {
    return SmartDashboard.getBoolean("UseTags", m_usetags);
  }

  static public boolean getShowtags() {
    return SmartDashboard.getBoolean("ShowTags", m_showtags);
  }

  static public boolean getUsePathplanner() {
    return SmartDashboard.getBoolean("Pathplanner", m_pathplanner);
  }

  public static void log(String msg) {
    if (!debug_commands)
      return;
    if (m_inAuto) {
      double tm = Drivetrain.autoTime();
      System.out.format("%-2.3f (%-2.3f) %s\n", tm, tm - last_time, msg);
      last_time = tm;
    }
    else
      System.out.format("%s\n", msg);
    SmartDashboard.putBoolean("OkToRun", okToRun());
  }

  public SequentialCommandGroup getCommand() {
    PlotUtils.auto_plot_option = m_auto_plot_option.getSelected();
    int auto_select= m_path_chooser.getSelected();
    SequentialCommandGroup acmnd=getAutoCommand(auto_select);
    if(acmnd==null){
      System.out.println("failed to create Auto sequence !");
      return null;
    }
    return new SequentialCommandGroup(new StartAuto(m_drive),acmnd,new EndAuto(m_drive));
  }
  SequentialCommandGroup startSequence() {
    return new SequentialCommandGroup(
        new GetStartPose(m_arm),
        //new AutoTarget(m_arm, m_drive),
        new Shoot(m_arm, m_drive));
  }
  
  SequentialCommandGroup twoNoteSequence(int pos) {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            new DrivePath(m_drive, pos, false),
            new Pickup(m_arm)), // sets angle to ground at start then to speaker when note captured
        new DrivePath(m_drive, pos, true),
        //new AutoTarget(m_arm, m_drive),
        new Shoot(m_arm, m_drive));
  }
  public SequentialCommandGroup getAutoCommand(int auto_select) {
    switch (auto_select) {
      case CALIBRATE:
        return new SequentialCommandGroup(new Calibrate(m_drive));
      case PROGRAM:
        return new SequentialCommandGroup(new DrivePath(m_drive, getReverse()));
      case ONE_NOTE: 
        return new SequentialCommandGroup(
          startSequence(),
          new ParallelCommandGroup(
              new DrivePath(m_drive, false),
              new Pickup(m_arm))
          );
      case TWO_NOTE: 
        return new SequentialCommandGroup(
            getAutoCommand(ONE_NOTE),
            new DrivePath(m_drive, true),
            //new AutoTarget(m_arm, m_drive),
            new Shoot(m_arm, m_drive)
            );
      case THREE_NOTE: 
        return new SequentialCommandGroup(
            startSequence(),
            twoNoteSequence(TargetMgr.CENTER),
            twoNoteSequence(TargetMgr.LEFT)
            );
       case FOUR_NOTE: 
        return new SequentialCommandGroup(
            getAutoCommand(THREE_NOTE),
            twoNoteSequence(TargetMgr.RIGHT)
            );
      case PATH: {    
        if (m_pathplanner) {
          int position = TargetMgr.getStartPosition();
          String pathname = "RightSideZeroed";
          if (position == TargetMgr.CENTER)
            pathname = "CenterZeroed";

          Pose2d p = PathPlannerAuto.getStaringPoseFromAutoFile(pathname);
          if (p != null)
            m_drive.resetOdometry(p);

          List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(pathname);
          return new SequentialCommandGroup(
              new Shoot(m_arm, m_drive),
              AutoBuilder.followPath(pathGroup.get(0)),
              new Pickup(m_arm),
              AutoBuilder.followPath(pathGroup.get(1)),
              new Shoot(m_arm, m_drive));
        } else { // using PathWeaver
          int placement = TargetMgr.getStartPlacement();
          String nameString="Center";
          if(placement==TargetMgr.RIGHT)
            nameString="Right";
          else if(placement==TargetMgr.LEFT)
            nameString="Left";         
          String forward = "paths/"+nameString+"Forward.wpilib.json";
          String reverse = "paths/"+nameString+"Reverse.wpilib.json";
          Path forwardPath = Filesystem.getDeployDirectory().toPath().resolve(forward);
          Path reversePath = Filesystem.getDeployDirectory().toPath().resolve(reverse);
          System.out.println("forward="+forwardPath);
          System.out.println("reverse="+reversePath);
          try {         
            Trajectory traj1 = TrajectoryUtil.fromPathweaverJson(forwardPath);           
            Trajectory traj2 = TrajectoryUtil.fromPathweaverJson(reversePath);
            Pose2d p0=traj1.getInitialPose();
            traj1=traj1.relativeTo(p0); // convert to robot ccordinates
            traj2=traj2.relativeTo(p0); // convert to robot ccordinates

            return new SequentialCommandGroup(
               new GetStartPose(m_arm),
               new AlignWheels(m_drive, 2),
               new Shoot(m_arm, m_drive),
               new ParallelCommandGroup(
                  new DrivePath(traj1,m_drive,false),
                  new Pickup(m_arm)),
              new DrivePath(traj2,m_drive,true),
              //new AutoTarget(m_arm, m_drive),
              new Shoot(m_arm, m_drive));         
          } catch (IOException ex) {
            System.out.println("Unable to open trajectory");
          }
        }
      }
    }
    return null;
  }
}
