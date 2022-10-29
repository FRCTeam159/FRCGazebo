// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
//import com.pathplanner.lib.PathPlannerTrajectory.Waypoint;

import com.pathplanner.lib.controllers.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Drivetrain;
import utils.PathData;
import utils.PlotUtils;

// =================================================
// DrivePath: class constructor (called from RobotContainer)
// =================================================
public class DrivePath extends CommandBase {
  /** Creates a new AutoTest. */
  private final ArrayList<PathData> pathdata = new ArrayList<PathData>();

  public boolean holotest=true;
  private final RamseteController m_ramsete = new RamseteController();
  private final PPHolonomicDriveController m_ppcontroller=new PPHolonomicDriveController(
      new PIDController(1, 0, 0), new PIDController(1, 0, 0), new PIDController(1, 0, 0));

// Here, our rotation profile constraints were a max velocity
// of 1 rotation per second and a max acceleration of 180 degrees
// per second squared.
  private final Timer m_timer = new Timer();
  private final Drivetrain m_drive;
  static public boolean plot_trajectory_motion = false;
  static public boolean plot_trajectory_dynamics = false;

  Trajectory m_trajectory;
  double runtime;
  double elapsed = 0;
  int states;
  int intervals;
  double yPath = 4;
  double xPath = 4;
  double rPath = 90;
  boolean reversed = false;

  double maxV;
  double maxA;
  double last_time;
  double angle;

  int plot_type = utils.PlotUtils.PLOT_NONE;
  int trajectory_option = Autonomous.PROGRAM;

  public DrivePath(Drivetrain drive, int opt,boolean rev) {
    reversed=rev;
    trajectory_option=opt;
    m_drive = drive;
    addRequirements(drive);
    maxV=Drivetrain.kMaxVelocity;
    maxA=Drivetrain.kMaxAcceleration;
  }

  // =================================================
  // initialize: Called when the command is initially scheduled.
  // =================================================
  @Override
  public void initialize() {
    plot_type = PlotUtils.auto_plot_option;
    System.out.println("DRIVEPATH_INIT");

    xPath = SmartDashboard.getNumber("xPath", xPath);
    yPath = SmartDashboard.getNumber("yPath", yPath);
    rPath = SmartDashboard.getNumber("rPath", rPath);

    holotest=SmartDashboard.getBoolean("test", false);

    angle=m_drive.getHeading();

    PlotUtils.initPlot();

    m_trajectory=getTrajectory();
    if(m_trajectory==null)
      return;

    PlotUtils.setInitialPose(m_trajectory.sample(0).poseMeters, Drivetrain.kTrackWidth);
    // PlotUtils.setDistanceUnits(PlotUtils.UnitType.FEET);

    runtime = m_trajectory.getTotalTimeSeconds();
    states = m_trajectory.getStates().size();
    intervals = (int) (runtime / 0.02);
    Pose2d p = m_trajectory.getInitialPose();

    m_drive.resetOdometry(p);

    m_timer.reset();
    m_timer.start();
    
    pathdata.clear();
    m_drive.startAuto();
    elapsed=0;
  
    System.out.println("runtime:" + runtime + " states:" + states + " intervals:" + intervals);
  }

  // =================================================
  // execute: Called every time the scheduler runs while the command is scheduled
  // =================================================
  @Override
  public void execute() {
    //elapsed = m_timer.get();
    elapsed = m_drive.getTime();
  
    Trajectory.State reference = m_trajectory.sample(elapsed);

    ChassisSpeeds speeds;

    if (trajectory_option == Autonomous.PATHPLANNER || holotest) {  // use pathplanner api
      speeds = m_ppcontroller.calculate(m_drive.getPose(), (PathPlannerState) reference);
      m_drive.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false);
    }
    else { // differential drive mode - no side drift
      speeds = m_ramsete.calculate(m_drive.getPose(), reference);
      m_drive.drive(speeds.vxMetersPerSecond, 0, speeds.omegaRadiansPerSecond, false);
    }
  
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
    System.out.println("DRIVEPATH_END");
    if (m_trajectory == null)
      return;
    m_drive.endAuto();
    //m_drive.reset();
    // m_drive.enable();
    if (plot_type != utils.PlotUtils.PLOT_NONE)
      utils.PlotUtils.publish(pathdata, 6, plot_type);
  }

  // =================================================
  // isFinished: Returns true when the command should end
  // =================================================
  @Override
  public boolean isFinished() {
    return (elapsed >= 1.001 * runtime || m_trajectory == null)||m_drive.disabled();
  }

  // *********************** trajectory functions *******************/

  // =================================================
  // getTrajectory: return a selected trajectory
  // =================================================
  Trajectory getTrajectory() {
    switch (trajectory_option) {
      case Autonomous.PROGRAM:
        return programPath();
      case Autonomous.PATHWEAVER:
        return pathWeaverTest();
      case Autonomous.PATHPLANNER:
        return pathPlannerTest();
    }
    return null;
  }

  Trajectory programPath(){
    if(holotest)
      return programPathPP();
    else
      return programPathPW();
  }
  // =================================================
  // programPath: build a multi-point trajectory from variables
  // =================================================
  // - lots of magic done when reverse is set in config
  // - inverts coordinate system from p.o.v. of robot's last pose
  //   (but facing in reverse direction)
  // - just need to reverse order of points to drive backwards
  // =================================================
  Trajectory programPathPW() {
    Pose2d pos1 = new Pose2d(0, 0, new Rotation2d(angle));
    Pose2d pos2 = new Pose2d(xPath, yPath, Rotation2d.fromDegrees(rPath));
   
    List<Pose2d> points = new ArrayList<Pose2d>();

    points.add(pos1);
    points.add(pos2);

    TrajectoryConfig config = new TrajectoryConfig(maxV, maxA);

    config.setReversed(reversed);
    if (reversed)
      Collections.reverse(points); // reverse order of waypoints first point=orig last point

    return TrajectoryGenerator.generateTrajectory(points, config);
  }

  // =================================================
  // programPathPP: build a PathBuilder trajectory from variables
  // =================================================
  Trajectory programPathPP() {
    PathPoint p1 = new PathPoint(new Translation2d(0.0, 0.0), new Rotation2d(0));
    PathPoint p2 = new PathPoint(new Translation2d(xPath, yPath), Rotation2d.fromDegrees(rPath), Rotation2d.fromDegrees(rPath));

    PathConstraints constraints= new PathConstraints(maxV, maxA);
    // reversal not supported for swerve drive 
    return PathPlanner.generatePath(constraints, reversed, p1, p2); 
  }

  // =================================================
  // pathWeaverTest: build a trajectory from PathWeaver data
  // =================================================
  Trajectory pathWeaverTest() {
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("Paths/Unnamed.wpilib.json");
      Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      return trajectory;
    } catch (IOException ex) {
      System.out.println("failed to create pathweaver trajectory");
      return null;
    }
  }

  // =================================================
  // pathPlannerTest: build a trajectory from PathPlanner data
  // =================================================
  Trajectory pathPlannerTest() {
    try {
      Trajectory trajectory = PathPlanner.loadPath("swervetest", 
        new PathConstraints(Drivetrain.kMaxVelocity,Drivetrain.kMaxAcceleration)); // max vel & accel
      return trajectory;
    } catch (Exception ex) {
      System.out.println("failed to create pathweaver trajectory");
      return null;
    }
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
        Drivetrain.kTrackWidth);
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

        Drivetrain.kTrackWidth);
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
}
