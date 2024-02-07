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
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import objects.PlotServer;
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

  boolean using_pathplanner=false;

  /** Creates a new AutoTest. */
  ArrayList<PathData> pathdata = new ArrayList<PathData>();

  public boolean holotest=true;
  final PPHolonomicDriveController m_ppcontroller = new PPHolonomicDriveController(
    new PIDConstants(4,0,0), new PIDConstants(4,0,0), Drivetrain.kMaxVelocity, 0.337);

  TrapezoidProfile.Constraints c=new TrapezoidProfile.Constraints(6.3, 3.15);
  ProfiledPIDController ppc=new ProfiledPIDController(4, 0, 0,c);
  HolonomicDriveController m_hcontroller=new HolonomicDriveController(new PIDController(1, 0, 0), new PIDController(1, 0, 0),ppc);
  
  Timer m_timer = new Timer();
  Drivetrain m_drive;
  static public boolean plot_trajectory_motion = false;
  static public boolean plot_trajectory_dynamics = false;

  Trajectory m_trajectory;
  PathPlannerTrajectory m_pptrajectory;
  double runtime;
  double elapsed = 0;
  int states;
  int intervals;
  double yPath = 4;
  double xPath = 4;
  double rPath = 90;
  boolean reversed = false;
  boolean autostart=false;
  double start_time;

  double maxV;
  double maxA;
  
  double last_time;

  int plot_type = utils.PlotUtils.PLOT_NONE;
  int trajectory_option = Autonomous.PROGRAM;

  public DrivePath(Drivetrain drive, int opt) {
    trajectory_option=opt;
    m_drive = drive;
    xPath = SmartDashboard.getNumber("xPath", xPath);
    yPath = SmartDashboard.getNumber("yPath", yPath);
    rPath = SmartDashboard.getNumber("rPath", rPath);
    addRequirements(drive);
  }

  public DrivePath(Drivetrain drive,int opt, boolean rev) {
    trajectory_option=opt;
    reversed=rev;
    m_drive = drive;
    autostart=true;
    addRequirements(drive);
  }
  // =================================================
  // initialize: Called when the command is initially scheduled.
  // =================================================
  @Override
  public void initialize() {
    plot_type = PlotUtils.auto_plot_option;
    System.out.println("DRIVEPATH_INIT");
    if(autostart){
      Pose2d target=TargetMgr.getTarget(reversed);
      xPath=target.getX();
      yPath=target.getY();
      rPath=target.getRotation().getRadians(); 
    }
    SmartDashboard.putNumber("xPath", xPath);
    SmartDashboard.putNumber("yPath", yPath);
    SmartDashboard.putNumber("rPath", rPath);

    maxV=Drivetrain.kMaxVelocity;
    maxA=Drivetrain.kMaxAcceleration;

    using_pathplanner=(trajectory_option == Autonomous.PATHPLANNER || trajectory_option == Autonomous.PROGRAMPP);
    if(xPath<0)
      reversed=true;
    else
      reversed=false;

    PlotUtils.initPlot();

    if(!getTrajectory()){
       System.out.println("failed to create Trajectory");
       return;
    }

    m_timer.reset();
    m_timer.start();
   
    pathdata.clear();
    
    elapsed=0;
    start_time=m_drive.getTime();
    m_drive.resetPose();
    //m_drive.enable();
    Arm.status="DrivePath";
  
    System.out.println("runtime:" + runtime + " states:" + states + " intervals:" + intervals);
  }

  // =================================================
  // execute: Called every time the scheduler runs while the command is scheduled
  // =================================================
  @Override
  public void execute() {
   
  elapsed = m_drive.getTime()-start_time;
  
   Trajectory.State reference=null;
    ChassisSpeeds speeds;
    if(using_pathplanner){
      PathPlannerTrajectory.State pstate = m_pptrajectory.sample(elapsed);
      speeds= m_ppcontroller.calculateRobotRelativeSpeeds(m_drive.getPose(), pstate);
      reference= new Trajectory.State(pstate.timeSeconds,pstate.velocityMps,pstate.accelerationMpsSq,pstate.getTargetHolonomicPose(),pstate.curvatureRadPerMeter);
    }
    else{
      reference = m_trajectory.sample(elapsed);
      speeds= m_hcontroller.calculate(m_drive.getPose(), reference,reference.poseMeters.getRotation());
    }
    m_drive.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, 2*speeds.omegaRadiansPerSecond, false);
  
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
   
    if (plot_type != utils.PlotUtils.PLOT_NONE)
      PlotServer.publish(pathdata, 6, plot_type);
  }

  // =================================================
  // isFinished: Returns true when the command should end
  // =================================================
  @Override
  public boolean isFinished() {
    return (elapsed >= 1.001 * runtime)||m_drive.disabled()||!Autonomous.ok2run;
  }

  // *********************** trajectory functions *******************/

  // =================================================
  // getTrajectory: return a selected trajectory
  // =================================================
  boolean getTrajectory() {
    switch (trajectory_option) {
      case Autonomous.PROGRAM:
        m_trajectory = programPath();
        if (m_trajectory == null) {
          return false;
        }
        runtime = m_trajectory.getTotalTimeSeconds();
        states = m_trajectory.getStates().size();
        PlotUtils.setInitialPose(m_trajectory.sample(0).poseMeters, Drivetrain.kTrackWidth);

        break;
      case Autonomous.PROGRAMPP:
        m_pptrajectory = pathplannerProgramPath();
        if (m_pptrajectory == null)
          return false;
        runtime = m_pptrajectory.getTotalTimeSeconds();
        states = m_pptrajectory.getStates().size();
        PlotUtils.setInitialPose(m_pptrajectory.sample(0).getTargetHolonomicPose(), Drivetrain.kTrackWidth);
        break;
      // case Autonomous.PATHPLANNER:
      // return pathPlannerTest();
      default:
        System.out.println("ERROR unknown trajectory type:" + trajectory_option);
        return false;
    }
    intervals = (int) (runtime / 0.02);
    return true;
  }

  // =================================================
  // programPathPP: build a PathBuilder trajectory from variables
  // =================================================
  PathPlannerTrajectory pathplannerProgramPath() {
    List<Pose2d> points = new ArrayList<Pose2d>();
    Pose2d pos1 = new Pose2d(0, 0, new Rotation2d(0));
    Pose2d pos2 = new Pose2d(xPath, yPath, Rotation2d.fromDegrees(rPath));

    points.add(pos1);
    points.add(pos2);
 
    if (reversed) 
      points=reverse(points);
    
    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(points);   
    PathConstraints constraints= new PathConstraints(maxV, maxA, Drivetrain.kMaxAngularSpeed, Drivetrain.kMaxAngularAcceleration);
  
    PathPlannerPath path = new PathPlannerPath(
        bezierPoints,constraints,
        new GoalEndState(0.0, Rotation2d.fromDegrees(rPath))
    );
   
    PathPlannerTrajectory traj=new PathPlannerTrajectory(path,new ChassisSpeeds(),new Rotation2d(0));
    return traj; 
  }

  // =================================================
  // reverse: reverse a set of points (drive backwards)
  // =================================================
  List<Pose2d> reverse(List<Pose2d> points){
    final var flip = new Transform2d(new Translation2d(), Rotation2d.fromDegrees(180.0));
    List<Pose2d> newWaypoints = new ArrayList<>();
    for (Pose2d originalWaypoint : points) 
      newWaypoints.add(originalWaypoint.plus(flip));     
    return newWaypoints;
 }
 
  // =================================================
  // programPath: build a two-point trajectory from variables
  // =================================================
  Trajectory programPath() {
    Pose2d pos1 = new Pose2d(0, 0, new Rotation2d(0));
    Pose2d pos2 = new Pose2d(xPath, yPath, Rotation2d.fromDegrees(rPath));

    List<Pose2d> points = new ArrayList<Pose2d>();

    points.add(pos1);
    points.add(pos2);
   
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
    TrajectoryConfig config = new TrajectoryConfig(maxV,maxA);
    System.out.println("reversed="+reversed);
    config.setReversed(reversed);
    //if (reversed)
    //  Collections.reverse(points); // reverse order of waypoints first point=orig last point
    return TrajectoryGenerator.generateTrajectory(points, config);
  }
  // =================================================
  // pathPlannerTest: build a trajectory from PathPlanner data
  // =================================================
  /* 
  Trajectory pathPlannerTest() {
    try {
      PathPlannerTrajectory trajectory = PathPlanner.loadPath("test", 
        new PathConstraints(Drivetrain.kMaxVelocity,Drivetrain.kMaxAcceleration)); // max vel & accel

      Pose2d p0 = trajectory.getInitialPose();

      // Pathplanner sets 0,0 as the lower left hand corner (FRC field coord system) 
      // for Gazebo, need to subtract intitial pose from each state so that 0,0 is 
      // in the center of the field 

      List<State> states = trajectory.getStates();
      for(int i=0;i<states.size();i++){
        PathPlannerTrajectory.PathPlannerState state=trajectory.getState(i);
        Pose2d p=state.poseMeters;
        Rotation2d h=state.holonomicRotation;
        Pose2d pr=p.relativeTo(p0);
        if(i==0)
         pr=new Pose2d(pr.getTranslation(),new Rotation2d()); // 
        state.holonomicRotation=h.plus(new Rotation2d(Math.toRadians(180))); // go backwards
        state.poseMeters=pr;
      }
      return trajectory;
    } catch (Exception ex) {
      System.out.println("failed to create pathweaver trajectory");
      return null;
    }
  }
  */
  
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
    // if(using_pathplanner)
    //   r=((PathPlannerState)state).holonomicRotation;
    // else
      r=state.poseMeters.getRotation();
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
    // if(using_pathplanner)
    //   pd.d[4] = 2*((PathPlannerState)state).holonomicRotation.getRadians();
    // else
      pd.d[4] = 2*state.poseMeters.getRotation().getRadians();
    pd.d[5] = 2*current_pose.getRotation().getRadians();

    pathdata.add(pd);
  }
}
