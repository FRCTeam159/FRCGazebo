// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.Calibrate;
import utils.PlotUtils;

public class Autonomous extends SequentialCommandGroup  {
  SendableChooser<Integer> m_auto_plot_option;

  
  DriveTrain m_drive;
 
  public static boolean debug_commands=false;

  /** Creates a new AutoCommands. 
   * @param m_claw*/
  public Autonomous() {
    m_drive=Robot.driveTrain;
   
    m_auto_plot_option = new SendableChooser<Integer>();
   
    m_auto_plot_option.setDefaultOption("No Plot", PlotUtils.PLOT_NONE);
    m_auto_plot_option.addOption("Plot Distance", PlotUtils.PLOT_DISTANCE);
    m_auto_plot_option.addOption("Plot Dynamics", PlotUtils.PLOT_DYNAMICS);
    m_auto_plot_option.addOption("Plot Position", PlotUtils.PLOT_POSITION);
    m_auto_plot_option.addOption("Calibrate", PlotUtils.PLOT_CALIBRATE);

    SmartDashboard.putData(m_auto_plot_option);
  }
  public SequentialCommandGroup  getCommand(){
    PlotUtils.auto_plot_option=m_auto_plot_option.getSelected();
    
    switch (PlotUtils.auto_plot_option){
    case PlotUtils.PLOT_CALIBRATE:
      return new SequentialCommandGroup(new Calibrate());
    default:
      return new AutonomousCommand();
    }
  }
}
