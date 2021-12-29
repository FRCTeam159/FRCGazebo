// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Trajectories;
import utils.PathData;
import utils.PlotUtils;

public class DrivePath extends CommandBase {
  /** Creates a new AutoTest. */
  ArrayList<PathData> pathdata = new ArrayList<PathData>();
  private final RamseteController m_ramsete = new RamseteController();
  private final Timer m_timer = new Timer();
  private final Drivetrain m_drive;
  static public boolean plot_calculated_path = false;
  static public boolean plot_motion = false;
  static public boolean plot_dynamics = false;
  static public boolean publish_path = false;
  static public boolean first_call = true;
  
  Trajectory m_trajectory;
  public DrivePath(Drivetrain subsystem) {
    m_drive=subsystem;
    addRequirements(subsystem);
    setTrajectory(Trajectories.curvedPath());
  }
  public void setTrajectory(Trajectory t){
    m_trajectory = t;
    if(first_call)
    if (plot_dynamics || plot_motion) {
      m_trajectory = m_trajectory.relativeTo(m_trajectory.getInitialPose());
      List<Trajectory.State> list = m_trajectory.getStates();
      if (plot_motion) 
        PlotUtils.plotPathMotion(list, Drivetrain.kTrackWidth);    
      if (plot_dynamics) 
        PlotUtils.plotPathDynamics(list);    
    }
    first_call=false;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.resetOdometry(m_trajectory.getInitialPose());
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double elapsed = m_timer.get();
    Trajectory.State reference = m_trajectory.sample(elapsed);
    ChassisSpeeds speeds = m_ramsete.calculate(m_drive.getPose(), reference);
    m_drive.odometryDrive(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
  }
  private void debugPath(double tm, Trajectory.State state){

  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
