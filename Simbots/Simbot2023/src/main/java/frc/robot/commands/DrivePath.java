// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import com.pathplanner.lib.controllers.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.objects.PlotServer;
import frc.robot.subsystems.TagDetector;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Drivetrain;
import utils.PathData;
import utils.PlotUtils;

// =================================================
// DrivePath: class constructor (called from RobotContainer)
// =================================================
public class DrivePath extends CommandBase {

  static boolean using_pathplanner=true;

  /** Creates a new AutoTest. */
  private ArrayList<PathData> pathdata = new ArrayList<PathData>();

  public boolean holotest=true;
  //private final RamseteController m_ramsete = new RamseteController();
  private final PPHolonomicDriveController m_ppcontroller=new PPHolonomicDriveController(
      new PIDController(1, 0, 0), new PIDController(1, 0, 0), new PIDController(0.5, 0, 0));

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

  int plot_type = utils.PlotUtils.PLOT_NONE;
  int trajectory_option = Autonomous.PROGRAM;

  public DrivePath(Drivetrain drive, int opt,boolean rev) {
    reversed=rev;
    trajectory_option=opt;
    m_drive = drive;
    addRequirements(drive);
    
  }

  // =================================================
  // initialize: Called when the command is initially scheduled.
  // =================================================
  @Override
  public void initialize() {
    plot_type = PlotUtils.auto_plot_option;
    System.out.println("DRIVEPATH_INIT");

    maxV=Drivetrain.kMaxVelocity;
    maxA=Drivetrain.kMaxAcceleration;

    xPath = SmartDashboard.getNumber("xPath", xPath);
    yPath = SmartDashboard.getNumber("yPath", yPath);
    rPath = SmartDashboard.getNumber("rPath", rPath);

    //holotest=SmartDashboard.getBoolean("holographic", true);

    using_pathplanner=(trajectory_option == Autonomous.PATHPLANNER || holotest);

    PlotUtils.initPlot();

    m_trajectory=getTrajectory();
    if(m_trajectory ==null){
      System.out.println("failed to create Trajectory");
      return;
    }
    
    PlotUtils.setInitialPose(m_trajectory.sample(0).poseMeters, Drivetrain.kTrackWidth);
    // PlotUtils.setDistanceUnits(PlotUtils.UnitType.FEET);

    runtime = m_trajectory.getTotalTimeSeconds();
    states = m_trajectory.getStates().size();
    intervals = (int) (runtime / 0.02);
    Pose2d p = m_trajectory.getInitialPose();

    m_drive.resetOdometry(p);

    //System.out.println(p);

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
    if(m_trajectory==null){
      System.out.print("ERROR DrivePath.execute - trajectory is null");
       return;
    }
    elapsed = m_drive.getTime();
  
    Trajectory.State reference = m_trajectory.sample(elapsed);

    ChassisSpeeds speeds = m_ppcontroller.calculate(m_drive.getPose(), (PathPlannerState) reference);
      m_drive.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false);
  
    if(plot_type == PlotUtils.PLOT_LOCATION)
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
    System.out.println("DRIVEPATH_END");
    if (m_trajectory == null)
      return;
    m_drive.endAuto();
    TagDetector.setBestTarget();

    //m_drive.reset();
    // m_drive.enable();
    if (plot_type != utils.PlotUtils.PLOT_NONE)
      PlotServer.publish(pathdata, 6, plot_type);
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
      case Autonomous.PATHPLANNER:
        return pathPlannerTest();
      default:
        System.out.println("ERROR unknown trajectory type:"+trajectory_option);
    }
    return null;
  }

  // =================================================
  // programPathPP: build a PathBuilder trajectory from variables
  // =================================================
  PathPlannerTrajectory programPath() {
    PathPoint p1 = new PathPoint(new Translation2d(0.0, 0.0), new Rotation2d(0));
    PathPoint p2 = new PathPoint(new Translation2d(xPath, yPath), Rotation2d.fromDegrees(rPath), Rotation2d.fromDegrees(rPath));

    PathConstraints constraints= new PathConstraints(maxV, maxA);
    // reversal not supported for swerve drive 
    PathPlannerTrajectory traj=PathPlanner.generatePath(constraints, reversed, p1, p2);

  
    // if(yPath>0 && TargetMgr.numTargets()>1)
    //   AprilTagDetector.setTargetId(1);
    // else
    //   AprilTagDetector.setTargetId(0);
    return traj; 
  }

  // =================================================
  // pathPlannerTest: build a trajectory from PathPlanner data
  // =================================================
  Trajectory pathPlannerTest() {
    try {
      PathPlannerTrajectory trajectory = PathPlanner.loadPath("swervetest", 
        new PathConstraints(Drivetrain.kMaxVelocity,Drivetrain.kMaxAcceleration)); // max vel & accel

      Pose2d p0 = trajectory.getInitialPose();

      // Pathplanner sets 0,0 as the lower left hand corner (FRC field coord system) 
      // for Gazebo, need to subtract intitial pose from each state so that 0,0 is 
      // in the center of the field 

      List<State> states = trajectory.getStates();
      for(int i=0;i<states.size();i++){
        State state=states.get(i);
        Pose2d psi=state.poseMeters.relativeTo(p0);
        state.poseMeters=psi;
      }
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
    // PathPlanner uses holonomicRotation for heading
    Rotation2d r=((PathPlannerState)state).holonomicRotation;
    Pose2d p=new Pose2d(state.poseMeters.getTranslation(),r);
    state.poseMeters=p;
    
    PathData pd = PlotUtils.plotPosition(
        state.timeSeconds,
        state.poseMeters,
        m_drive.getPose(),
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
    Pose2d target_pose=state.poseMeters;
    Pose2d current_pose=m_drive.getPose();

    pd.d[0] = target_pose.getX();
    pd.d[1] = current_pose.getX();
    pd.d[2] = target_pose.getY();
    pd.d[3] = current_pose.getY();
    pd.d[4] = 2*((PathPlannerState)state).holonomicRotation.getRadians();
    pd.d[5] = 2*current_pose.getRotation().getRadians();

    pathdata.add(pd);
  }
}
