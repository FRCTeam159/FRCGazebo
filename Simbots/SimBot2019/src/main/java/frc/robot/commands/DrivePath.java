// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;
import utils.PathData;
import utils.PlotUtils;
import objects.PlotServer;

// =================================================
// DrivePath: class constructor (called from RobotContainer)
// =================================================
public class DrivePath extends CommandBase {
  /** Creates a new AutoTest. */
  private final ArrayList<PathData> pathdata = new ArrayList<PathData>();
  //private final RamseteController m_ramsete = new RamseteController(5,0.5);
  private final RamseteController m_ramsete = new RamseteController();

  private final DriveTrain m_drive;
  static public boolean plot_trajectory_motion = false;
  static public boolean plot_trajectory_dynamics = false;
  private String last_msg;

  Trajectory m_trajectory;
  double runtime;
  double elapsed = 0;
  int states;
  int intervals;
  //Timer m_timer=new Timer();

  boolean reversed = false;

  int plot_type = utils.PlotUtils.PLOT_NONE;

  public DrivePath() {
    m_drive = Robot.driveTrain;
    //m_timer.start();
    addRequirements(m_drive);
  }

  // =================================================
  // initialize: Called when the command is initially scheduled.
  // =================================================
  @Override
  public void initialize() {
    plot_type = PlotUtils.auto_plot_option;
    showStatus("DrivePath.start");

    PlotUtils.initPlot();

    m_trajectory=programPath();
    if(m_trajectory==null)
      return;

    Robot.simulation.reset();
    Robot.simulation.startAuto();

    PlotUtils.setInitialPose(m_trajectory.sample(0).poseMeters, DriveTrain.kTrackWidth);

    runtime = m_trajectory.getTotalTimeSeconds();
    states = m_trajectory.getStates().size();
    intervals = (int) (runtime / 0.02);
    Pose2d p = m_trajectory.getInitialPose();

    m_drive.resetOdometry(p);

    m_drive.enable();

    pathdata.clear();
    //m_drive.startAuto();
    elapsed=0;

    System.out.println("runtime:" + runtime + " states:" + states + " intervals:" + intervals);
  }

  // =================================================
  // execute: Called every time the scheduler runs while the command is scheduled
  // =================================================
  @Override
  public void execute() {
    elapsed = Robot.simulation.getClockTime();
    if (elapsed < 0.02)
      return;

    Trajectory.State reference = m_trajectory.sample(elapsed);

    ChassisSpeeds speeds = m_ramsete.calculate(m_drive.getPose(), reference);
    m_drive.odometryDrive(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
    if (plot_type == PlotUtils.PLOT_DISTANCE)
      plotDistance(reference);
    else if (plot_type == PlotUtils.PLOT_DYNAMICS)
      plotDynamics(reference);
    else if (plot_type == PlotUtils.PLOT_POSITION)
      plotPosition(reference);
  }

  // =================================================
  // end: Called once the command ends or is interrupted.
  // =================================================
  @Override
  public void end(boolean interrupted) {
    showStatus("DrivePath.end");
    if (m_trajectory == null)
      return;
    m_drive.reset();

    if (plot_type != utils.PlotUtils.PLOT_NONE)
      PlotServer.publish(pathdata, 6, plot_type);
  }

  // =================================================
  // isFinished: Returns true when the command should end
  // =================================================
  @Override
  public boolean isFinished() {
    return elapsed >= 1.001 * runtime || m_trajectory == null;
  }

  // =================================================
  // programPath: build a two-point trajectory from variables
  // =================================================
  Trajectory programPath() {
    Rotation2d z=new Rotation2d(0);
    Rotation2d t=new Rotation2d(Units.degreesToRadians(30));
    Pose2d pos1 = new Pose2d(0, 0, z);
    Pose2d pos2 = new Pose2d(3.0, 0.7, t);
    Pose2d pos3 = new Pose2d(4.5, 0.9, z);

    List<Pose2d> points = new ArrayList<Pose2d>();

    points.add(pos1);
    points.add(pos2);
    points.add(pos3);
   
    return makeTrajectory(points, reversed);
  }

  // =================================================
  // makeTrajectory: build a trajectory from a list of poses
  // =================================================
  // - lots of magic done when reverse is set in config
  // - inverts coordinate system from p.o.v. of robot's last pose
  //   (but facing in reverse direction)
  // - just need to reverse order of points to drive backwards
  // =================================================
  Trajectory makeTrajectory(List<Pose2d> points, boolean reversed) {
    TrajectoryConfig config = new TrajectoryConfig(DriveTrain.kMaxVelocity, DriveTrain.kMaxAcceleration);
    config.setReversed(reversed);
    if (reversed)
      Collections.reverse(points); // reverse order of waypoints first point=orig last point

    return TrajectoryGenerator.generateTrajectory(points, config);
  }

  // *********************** plotting methods *******************/

  // =================================================
  // plotPosition: collect PathData for motion error plot
  // =================================================
  // arg[0] time of sample
  // arg[1] target pose
  // arg[2] observed pose
  // arg[3] trackwidth
  // =================================================
  void plotPosition(Trajectory.State state) {
    PathData pd = PlotUtils.plotPosition(
        state.timeSeconds,
        state.poseMeters,
        m_drive.getPose(),
        DriveTrain.kTrackWidth);
    pathdata.add(pd);
  }

  // =================================================
  // plotDistance: collect PathData for distance error plot
  // =================================================
  // arg[0] time of sample
  // arg[1] target pose
  // arg[2] observed left distance
  // arg[3] observed right distance
  // arg[4] observed heading
  // arg[5] trackwidth
  // =================================================
  void plotDistance(Trajectory.State state) {
    double h = reversed ? -1 : 1;
    PathData pd = PlotUtils.plotDistance(
        state.timeSeconds,
        state.poseMeters,
        state.curvatureRadPerMeter,

        h * m_drive.getLeftDistance(),
        h * m_drive.getRightDistance(),
        m_drive.getHeading(),

        DriveTrain.kTrackWidth);
    pathdata.add(pd);
  }

  // =================================================
  // plotDynamics: collect PathData for dynamics error plot
  // =================================================
  // arg[0] time of sample
  // arg[1] target pose
  // arg[2] observed pose
  // arg[3] target velocity
  // arg[4] observed velocity
  // arg[5] target acceleration
  // =================================================
  void plotDynamics(Trajectory.State state) {
    PathData pd = PlotUtils.plotDynamics(
        state.timeSeconds,
        state.poseMeters, m_drive.getPose(),
        state.velocityMetersPerSecond, m_drive.getVelocity(),
        state.accelerationMetersPerSecondSq);
    pathdata.add(pd);
  }
  private void showStatus(String msg){
    SmartDashboard.putString("Status", msg);
    if(!msg.equals(last_msg))
      System.out.println(msg);
    last_msg=msg;
  }
}
