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
import utils.PathData;
import utils.PlotUtils;

public class DrivePath extends CommandBase {
  /** Creates a new AutoTest. */
  ArrayList<PathData> pathdata = new ArrayList<PathData>();
  private final RamseteController m_ramsete = new RamseteController();
  private final Timer m_timer = new Timer();
  private final Drivetrain m_drive;
  static public boolean plot_trajectory_motion = false;
  static public boolean plot_trajectory_dynamics = false;

  Trajectory m_trajectory;
  int m_type;

  double runtime;
  double elapsed = 0;
  int states;
  int intervals;

  int plot_type=utils.PlotUtils.PLOT_NONE;

  public DrivePath(Drivetrain drive) {
    m_drive = drive;
    addRequirements(drive);

  }
 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    plot_type=PlotUtils.auto_plot_option;
    m_type=m_drive.selected_path;
    System.out.println("path="+m_type);

    PlotUtils.initPlot();
    
    m_trajectory=genericPath(m_drive.xPath,m_drive.yPath,m_drive.rPath);
    
    PlotUtils.setInitialPose(m_trajectory.sample(0).poseMeters,Drivetrain.kTrackWidth);
    //PlotUtils.setDistanceUnits(PlotUtils.UnitType.FEET);

    runtime = m_trajectory.getTotalTimeSeconds();
    states = m_trajectory.getStates().size();
    intervals = (int) (runtime / 0.02);

    m_drive.resetOdometry(m_trajectory.getInitialPose());
    m_timer.reset();
    m_timer.start();

    pathdata.clear();
    m_drive.startAuto();
    System.out.println("runtime:" + runtime + " states:" + states + " intervals:" + intervals);
  }
 
  // Called every time the scheduler runs while the command is scheduled.\n
  @Override
  public void execute() {
    // elapsed = m_timer.get();
    elapsed = m_drive.getTime();
    if(elapsed<0.02)
      return;

    Trajectory.State reference = m_trajectory.sample(elapsed);
  
    ChassisSpeeds speeds = m_ramsete.calculate(m_drive.getPose(), reference);
    m_drive.odometryDrive(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);

    if (plot_type == PlotUtils.PLOT_DISTANCE)
      plotDistance(reference);
    else if (plot_type  == PlotUtils.PLOT_DYNAMICS)
      plotDynamics(reference);
    else if (plot_type  == PlotUtils.PLOT_POSITION)
      plotPosition(reference);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (m_trajectory == null)
      return;
    System.out.println("Simtime=" + m_drive.getTime() + " Realtime=" + runtime);
    if (plot_type != utils.PlotUtils.PLOT_NONE)
      utils.PlotUtils.publish(pathdata,6,plot_type);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elapsed >= 1.01*runtime || m_trajectory == null;
  }

  public Trajectory genericPath(double xVal, double yVal, double rVal) {
    Trajectory traj = TrajectoryGenerator.generateTrajectory
    (
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(), 
      new Pose2d(xVal, yVal, Rotation2d.fromDegrees(rVal)),
      new TrajectoryConfig(Drivetrain.kMaxVelocity, Drivetrain.kMaxAcceleration)
    );
    return traj;
  }
  // =================================================
  // plotPosition: collect PathData for motion error plot
  // =================================================
  // arg[0]  time of sample
  // arg[1]  target pose
  // arg[2]  observed pose
  // arg[3]  trackwidth
  // =================================================
  private void plotPosition(Trajectory.State state) {
  PathData pd=PlotUtils.plotPosition(
    state.timeSeconds,
    state.poseMeters,
    m_drive.getPose(),
    Drivetrain.kTrackWidth
    );
    pathdata.add(pd);
  }

  // =================================================
  // plotDistance: collect PathData for distance error plot
  // =================================================
  // arg[0]  time of sample
  // arg[1]  target pose
  // arg[2]  observed left distance
  // arg[3]  observed right distance
  // arg[4]  observed heading
  // arg[5]  trackwidth
  // =================================================
 private void plotDistance(Trajectory.State state) {
    PathData pd = PlotUtils.plotDistance(
    state.timeSeconds, 
    state.poseMeters, 
    m_drive.getLeftDistance(), 
    m_drive.getRightDistance(), 
    m_drive.getHeading(),
    Drivetrain.kTrackWidth);
    pathdata.add(pd);
  }
  // =================================================
  // plotDynamics: collect PathData for dynamics error plot
  // =================================================
  // arg[0]  time of sample
  // arg[1]  target pose
  // arg[2]  observed pose
  // arg[3]  target velocity
  // arg[4]  observed velocity
  // arg[5]  target acceleration
  // =================================================
   private void plotDynamics(Trajectory.State state) {
    PathData pd = PlotUtils.plotDynamics(
        state.timeSeconds,
        state.poseMeters, m_drive.getPose(),
        state.velocityMetersPerSecond, m_drive.getVelocity(),
        state.accelerationMetersPerSecondSq);
    pathdata.add(pd);
  }
}
