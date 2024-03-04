// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.TargetMgr;
import utils.PathData;
import utils.PlotUtils;

// =================================================
// DrivePath: class constructor (called from RobotContainer)
// =================================================
public class DrivePath extends Command {

  double scale = 1;

  ArrayList<PathData> pathdata = new ArrayList<PathData>();

  final PPHolonomicDriveController m_ppcontroller = new PPHolonomicDriveController(
      new PIDConstants(4, 0.0, 0), new PIDConstants(6, 0.0, 0.0), scale * Drivetrain.kMaxVelocity,
      Drivetrain.kTrackRadius);

  final HolonomicDriveController m_hcontroller = new HolonomicDriveController(
      new PIDController(2, 0, 0),
      new PIDController(2, 0, 0),
      new ProfiledPIDController(7, 0, 0,
          new TrapezoidProfile.Constraints(scale * Drivetrain.kMaxAngularVelocity,scale * Drivetrain.kMaxAngularAcceleration)));

  Timer m_timer = new Timer();
  Drivetrain m_drive;
  static public boolean plot_trajectory_motion = false;
  static public boolean plot_trajectory_dynamics = false;

  static boolean debug = false;

  PathPlannerTrajectory m_pptrajectory;
  Trajectory m_trajectory;
  boolean using_pathplanner = false;
  double runtime;
  double elapsed = 0;
  int states;
  int intervals;
  double yPath = 1;
  double xPath = 0;
  double rPath = 0;
  boolean m_reversed = false;
  boolean m_autoset = false;

  double last_heading = 0;
  boolean m_note_at_start;

  int plot_type = utils.PlotUtils.PLOT_NONE;
  double start_time;

  double cnt;

  public DrivePath(Drivetrain drive, boolean rev) {
    m_reversed = rev;
    m_drive = drive;
    addRequirements(drive);
  }

  public DrivePath(Trajectory traj, Drivetrain drive, boolean rev) {
    m_reversed = rev;
    m_drive = drive;
    m_trajectory = traj;
    addRequirements(drive);
  }

  // =================================================
  // initialize: Called when the command is initially scheduled.
  // =================================================
  @Override
  public void initialize() {
    Autonomous.log("Drivepath.init");
    plot_type = PlotUtils.auto_plot_option;
    m_autoset = Autonomous.getAutoset();

    using_pathplanner = Autonomous.getUsePathplanner();

    m_note_at_start = Arm.noteAtIntake();

    cnt = 0;

    if (m_autoset) { // use apriltags or smartdashboard buttons
      Pose2d target = TargetMgr.getTarget();
      xPath = target.getX();
      yPath = target.getY();
      rPath = target.getRotation().getRadians();

      SmartDashboard.putNumber("xPath", xPath);
      SmartDashboard.putNumber("yPath", yPath);
      SmartDashboard.putNumber("rPath", rPath);
    } else { // set path manually
      xPath = SmartDashboard.getNumber("xPath", xPath);
      yPath = SmartDashboard.getNumber("yPath", yPath);
      rPath = SmartDashboard.getNumber("rPath", rPath);
    }

    PlotUtils.initPlot();

    if (!getTrajectory()) {
      Autonomous.log("DrivePath - failed to create Trajectory");
      Autonomous.stop();
      return;
    }
    m_timer.start();
    m_timer.reset();

    pathdata.clear();

    elapsed = 0;
    start_time = m_drive.getTime();

    // important ! do this otherwise get rotation glitch at start or reverse path
    if (using_pathplanner)
      m_ppcontroller.reset(m_drive.getPose(), new ChassisSpeeds());
    Robot.status = "DrivePath";

    System.out.println("runtime:" + runtime + " states:" + states + " intervals:" + intervals);
  }

  // =================================================
  // execute: Called every time the scheduler runs while the command is scheduled
  // =================================================
  @Override
  public void execute() {
    if (!Autonomous.okToRun())
      return;

    elapsed = m_drive.getTime() - start_time;

    Trajectory.State reference = null;
    ChassisSpeeds speeds;

    if (using_pathplanner) {
      PathPlannerTrajectory.State pstate = m_pptrajectory.sample(elapsed);
      speeds = m_ppcontroller.calculateRobotRelativeSpeeds(m_drive.getPose(), pstate);
      reference = new Trajectory.State(pstate.timeSeconds, pstate.velocityMps, pstate.accelerationMpsSq,
          pstate.getTargetHolonomicPose(), pstate.curvatureRadPerMeter);
    } else {
      reference = m_trajectory.sample(elapsed);
      double angle = Drivetrain.unwrap(last_heading, reference.poseMeters.getRotation().getDegrees());
      last_heading = angle;
      Rotation2d rot = Rotation2d.fromDegrees(angle);
      reference.poseMeters = new Pose2d(reference.poseMeters.getTranslation(), rot);
      speeds = m_hcontroller.calculate(m_drive.getPose(), reference, rot);
    }
    // if (debug) {
    if (debug && (cnt % 10 == 0 || cnt == 0 || cnt == states - 1)) {
      Pose2d p = m_drive.getPose();
      System.out.format(
          "%-1.3f X a:%-3.1f t:%-3.1f c:%-3.1f Y a:%-1.1f t:%-1.1f c:%-3.1f R a:%-3.1f t:%-3.1f\n", elapsed,
          p.getTranslation().getX(), reference.poseMeters.getX(),speeds.vxMetersPerSecond,
          p.getTranslation().getY(), reference.poseMeters.getY(),speeds.vyMetersPerSecond,
          p.getRotation().getDegrees(), reference.poseMeters.getRotation().getDegrees());
    }
    cnt++;

    m_drive.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false);

    if (plot_type == PlotUtils.PLOT_LOCATION)
      plotLocation(reference);
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
    Autonomous.log("Drivepath.end");
    if (using_pathplanner)
      m_ppcontroller.setEnabled(false);
    else
      m_hcontroller.setEnabled(false);

    if (plot_type != utils.PlotUtils.PLOT_NONE)
      utils.PlotUtils.publish(pathdata, 6, plot_type);
  }

  // =================================================
  // isFinished: Returns true when the command should end
  // =================================================
  @Override
  public boolean isFinished() {
    if (!Autonomous.okToRun())
      return true;
    //if (!m_reversed && !m_note_at_start && Arm.noteAtIntake())
    //  return true;
    return (elapsed >= 1.2 * runtime);
  }

  // *********************** trajectory functions *******************/
  // ===============================================
  // programPathPP: build a PathBuilder trajectory from variables
  // =================================================
  PathPlannerTrajectory pathplannerProgramPath() {
    List<Pose2d> points = new ArrayList<Pose2d>();
    double rpg = 0;
    double rps = 0;

    if (m_reversed) { // go back to 0,0 !
      Pose2d pose = m_drive.getPose(); // start at current robot pose
      rps = m_drive.getHeading();
      points.add(pose);
      points.add(new Pose2d()); // end at zero
    } else {
      rpg = rPath;
      points.add(new Pose2d()); // start at zero
      points.add(new Pose2d(xPath, yPath, Rotation2d.fromDegrees(rPath)));
    }

    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(points);
    PathConstraints constraints = new PathConstraints(
        scale * Drivetrain.kMaxVelocity,
        scale * Drivetrain.kMaxAcceleration,
        scale * Drivetrain.kMaxAngularVelocity,
        scale * Drivetrain.kMaxAngularAcceleration);

    PathPlannerPath path = new PathPlannerPath(
        bezierPoints, constraints,
        new GoalEndState(0.0, Rotation2d.fromDegrees(rpg)));

    PathPlannerTrajectory traj = new PathPlannerTrajectory(path, new ChassisSpeeds(), Rotation2d.fromDegrees(rps));
    return traj;
  }

  // =================================================
  // programPath: build a trajectory for pathweaver
  // - use the trajectory passed in via constructor
  // =================================================
  Trajectory pathweaverPath() {
    //Pose2d p0=m_trajectory.getInitialPose();
    //Trajectory traj=m_trajectory.relativeTo(p0); // convert to robot ccordinates
    Trajectory traj=m_trajectory;
     
    if(!m_reversed){
      //m_drive.setPose(p0);
      List<State> states=traj.getStates();
      Pose2d s = states.get(states.size()-1).poseMeters;
      SmartDashboard.putNumber("xPath", s.getTranslation().getX());
      SmartDashboard.putNumber("yPath", s.getTranslation().getY());
      SmartDashboard.putNumber("rPath", s.getRotation().getDegrees());
    }
    // TrajectoryConfig config = new TrajectoryConfig(scale * Drivetrain.kMaxVelocity,
    //     scale * Drivetrain.kMaxAcceleration);
    // config.setKinematics(m_drive.getKinematics());
    // config.setReversed(m_reversed);
    return traj;
  }
  // =================================================
  // programPath: build a two-point trajectory from variables
  // =================================================
  Trajectory programPath() {
    List<Pose2d> points = new ArrayList<Pose2d>();
    Pose2d start;
    Pose2d end;

    if (m_reversed) { // go back to 0,0 !
      start = m_drive.getPose(); // start at current robot pose;
      end = new Pose2d();
    } else {
      start = new Pose2d(); // start at 0,0
      end = new Pose2d(xPath, yPath, Rotation2d.fromDegrees(rPath));
    }
    points.add(start);
    points.add(end);
    TrajectoryConfig config = new TrajectoryConfig(scale * Drivetrain.kMaxVelocity,
        scale * Drivetrain.kMaxAcceleration);
    config.setKinematics(m_drive.getKinematics());
    config.setReversed(m_reversed);
    return TrajectoryGenerator.generateTrajectory(points, config);
  }

  // =================================================
  // getTrajectory: return a selected trajectory
  // =================================================
  boolean getTrajectory() {
    Pose2d pose = m_drive.getPose();
    if (m_reversed && (pose.getX() < 0.2 && pose.getY()<0.2)) // probably an error to start too close to 0 ?
      return false;
    else if (!m_reversed && (pose.getX() > 0.2 || pose.getY()>0.2)) // probably an error to start too far from 0 ?
      return false;
    if (using_pathplanner) {
      m_pptrajectory = pathplannerProgramPath();
      if (m_pptrajectory == null)
        return false;
      runtime = m_pptrajectory.getTotalTimeSeconds();
      states = m_pptrajectory.getStates().size();
      PlotUtils.setInitialPose(m_pptrajectory.sample(0).getTargetHolonomicPose(), Drivetrain.kFrontWheelBase);
      m_ppcontroller.setEnabled(true);
    } else {
      if(m_trajectory==null)
        m_trajectory = programPath();
      else
        m_trajectory = pathweaverPath();

      if (m_trajectory == null)
        return false;
      runtime = m_trajectory.getTotalTimeSeconds();
      states = m_trajectory.getStates().size();
      PlotUtils.setInitialPose(m_trajectory.sample(0).poseMeters, Drivetrain.kFrontWheelBase);
      m_hcontroller.setEnabled(true);
    }
    intervals = (int) (runtime / 0.02);
    return true;
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
    // PathPlanner uses holonomicRotation for heading
    Rotation2d r;

    r = state.poseMeters.getRotation();
    Pose2d p = new Pose2d(state.poseMeters.getTranslation(), r);
    state.poseMeters = p;

    PathData pd = PlotUtils.plotPosition(
        state.timeSeconds,
        state.poseMeters,
        m_drive.getPose(),
        Drivetrain.kFrontWheelBase);
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

  // =================================================
  // plotLocation: collect PathData for location error plot
  // =================================================
  // arg[0] time of sample
  // arg[1] target X
  // arg[2] observed X
  // arg[3] target Y
  // arg[4] observed Y
  // arg[5] target Heading
  // arg[5] observed Heading
  void plotLocation(Trajectory.State state) {
    PathData pd = new PathData();
    pd.tm = state.timeSeconds;
    Pose2d target_pose = state.poseMeters;
    Pose2d current_pose = m_drive.getPose();

    pd.d[0] = target_pose.getX();
    pd.d[1] = current_pose.getX();
    pd.d[2] = target_pose.getY();
    pd.d[3] = current_pose.getY();

    pd.d[4] = 2 * state.poseMeters.getRotation().getRadians();
    pd.d[5] = 2 * current_pose.getRotation().getRadians();

    pathdata.add(pd);
  }
}
