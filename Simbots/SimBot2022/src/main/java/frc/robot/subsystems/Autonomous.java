// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.Calibrate;
import frc.robot.commands.DrivePath;
import frc.robot.commands.DriveToCargo;
import frc.robot.commands.DriveToTarget;
import utils.PlotUtils;

public class Autonomous extends SubsystemBase {
  SendableChooser<Integer> m_auto_plot_option = new SendableChooser<>();

  private final DriveTrain m_drive;
  private final Targeting m_targeting;
  private final Shooting m_shoot;

  public static final int CALIBRATE = 0;
  public static final int PROGRAM = 1;
  public static final int DRIVEBACK = 3;
  public static final int ONEBALL = 4;
  public static final int TWOBALL = 5;
  public static final int THREEBALL = 6;

  public static double totalRuntime;
  public static boolean autoFailed = false;
  public static boolean autoFinished = false;

  public int selected_path=PROGRAM;

  SendableChooser<Integer> m_path_chooser = new SendableChooser<Integer>();
  /** Creates a new AutoCommands. */
  public Autonomous(DriveTrain drive,Targeting targeting,Shooting shoot) {
    m_drive=drive;
    m_targeting=targeting;
    m_shoot=shoot;
   
    m_auto_plot_option.setDefaultOption("No Plot", PlotUtils.PLOT_NONE);
    m_auto_plot_option.addOption("Plot Distance", PlotUtils.PLOT_DISTANCE);
    m_auto_plot_option.addOption("Plot Dynamics", PlotUtils.PLOT_DYNAMICS);
    m_auto_plot_option.addOption("Plot Position", PlotUtils.PLOT_POSITION);

    m_path_chooser.setDefaultOption("Program", PROGRAM);
    m_path_chooser.addOption("Calibrate", CALIBRATE);

    m_path_chooser.addOption("DriveBack", DRIVEBACK);
	  m_path_chooser.addOption("OneBall", ONEBALL);
    m_path_chooser.addOption("TwoBall", TWOBALL);
    m_path_chooser.addOption("ThreeBall", THREEBALL);

    SmartDashboard.putBoolean("reversed", false);
    SmartDashboard.putNumber("xPath", 4);
    SmartDashboard.putNumber("yPath", 0);
    SmartDashboard.putNumber("rPath", 0);
    
		SmartDashboard.putData(m_path_chooser);
    SmartDashboard.putData(m_auto_plot_option);
  }

  public CommandGroupBase getCommand(){
    PlotUtils.auto_plot_option=m_auto_plot_option.getSelected();
    selected_path=m_path_chooser.getSelected();
    
    //CommandGroupBase.clearGroupedCommands();
    totalRuntime=0;
    autoFailed=false;
    autoFinished=false;

    switch (selected_path){
    case CALIBRATE:
      return new SequentialCommandGroup(new Calibrate(m_drive));
    case PROGRAM:
      return programPath();
    case DRIVEBACK:
      return driveBack();
    case ONEBALL:
      return oneBall();
    case TWOBALL:
      return twoBall();
    case THREEBALL:
      return threeBall();
    }
    return null;
  }
  CommandGroupBase programPath(){
    double x = SmartDashboard.getNumber("xPath", 4);
    double y = SmartDashboard.getNumber("yPath", 0);
    double r = SmartDashboard.getNumber("rPath", 0);
    boolean rev=SmartDashboard.getBoolean("reversed", false);
    SequentialCommandGroup commands = new SequentialCommandGroup();
    commands.addCommands(new DrivePath(m_drive,x,y,r,rev));
    return commands;
  }
  private CommandGroupBase driveBack() {
    SequentialCommandGroup commands = new SequentialCommandGroup();
    commands.addCommands(new DrivePath(m_drive,1.5,0,0,true));
    return commands;
  }

  CommandGroupBase oneBall(){
    SequentialCommandGroup commands = new SequentialCommandGroup();
    commands.addCommands(new DriveToTarget(m_targeting,m_shoot));
    return commands;
  }
  CommandGroupBase twoBall(){
    SequentialCommandGroup commands = new SequentialCommandGroup();
    commands.addCommands(new DriveToTarget(m_targeting,m_shoot));
    commands.addCommands(new DriveToCargo(m_targeting,m_shoot));
    commands.addCommands(new DriveToTarget(m_targeting,m_shoot));
    return commands;
  }
  CommandGroupBase threeBall(){
    SequentialCommandGroup commands = new SequentialCommandGroup();
    commands.addCommands(new DriveToTarget(m_targeting,m_shoot));
    commands.addCommands(new DriveToCargo(m_targeting,m_shoot));
    commands.addCommands(new DriveToTarget(m_targeting,m_shoot));
    commands.addCommands(new DriveToCargo(m_targeting,m_shoot));
    //commands.addCommands(new DrivePath(m_drive,2,0,0,false));
    commands.addCommands(new DriveToTarget(m_targeting,m_shoot));
    return commands;
  }
  public static void setAutoStatus(){
    SmartDashboard.putNumber("Auto time",totalRuntime);
    SmartDashboard.putBoolean("Auto failed", autoFailed);
  }
  @Override
  public void periodic() {
    setAutoStatus();
  }
}
