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
import frc.robot.subsystems.TargetMgr;
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

  private final PPHolonomicDriveController m_ppcontroller=new PPHolonomicDriveController(
      new PIDController(5, 0, 0), new PIDController(4, 0, 0.0), new PIDController(3, 0.0, 0.0));

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

  double maxV;
  double maxA;
  double last_time;
  double last_heading;
  int cnt=0;

  static boolean debug_path=false;
  int plot_type = utils.PlotUtils.PLOT_NONE;
  int trajectory_option = Autonomous.PROGRAM;

  public DrivePath(Drivetrain drive, int opt) {
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

    using_pathplanner=(trajectory_option == Autonomous.PATHPLANNER);

    PlotUtils.initPlot();

    m_trajectory=getTrajectory();
    if(m_trajectory ==null){
      System.out.println("failed to create Trajectory");
      return;
    }
    
    PlotUtils.setInitialPose(m_trajectory.sample(0).poseMeters, Drivetrain.kTrackWidth);

    runtime = m_trajectory.getTotalTimeSeconds();
    states = m_trajectory.getStates().size();
    intervals = (int) (runtime / 0.02);
    Pose2d p = m_trajectory.getInitialPose();

    elapsed=0;
    PathPlannerState reference = (PathPlannerState)m_trajectory.sample(0);
    last_heading=reference.holonomicRotation.getDegrees();

    m_drive.resetOdometry(p); 

    m_timer.reset();
    m_timer.start();
    
    pathdata.clear();
    m_drive.startAuto();

  
    System.out.println("runtime:" + runtime + " states:" + states + " intervals:" + intervals);
  }

  // =================================================
  // execute: Called every time the scheduler runs while the command is scheduled
  // =================================================
  @Override
  public void execute() {
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

    if (plot_type != utils.PlotUtils.PLOT_NONE)
      PlotServer.publish(pathdata, 6, plot_type);
  }

  // =================================================
  // isFinished: Returns true when the command should end
  // =================================================
  @Override
  public boolean isFinished() {
    //return true;
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
        return pathPlannerPath();
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
    PathPlannerTrajectory traj=PathPlanner.generatePath(constraints, false, p1, p2);

    if(!TargetMgr.FRCfield()){
      if(yPath>0 && TargetMgr.numTargets()>1)
        TagDetector.setTargetId(1);
      else
        TagDetector.setTargetId(0);
    }
    return traj; 
  }

  // =================================================
  // pathPlannerTest: build a trajectory from PathPlanner data
  // =================================================
  Trajectory pathPlannerPath() {
    try {
      String file="swervetest";
      if(TargetMgr.FRCfield()){
        if(TargetMgr.getStartPosition()==TargetMgr.OUTSIDE)
          file="LeftAuto";
        else if(TargetMgr.getStartPosition()==TargetMgr.INSIDE)
          file="RightAuto";
        else if(TargetMgr.getStartPosition()==TargetMgr.CENTER)
          file="CenterAuto";
      }
      PathPlannerTrajectory trajectory = PathPlanner.loadPath(file, 
        new PathConstraints(Drivetrain.kMaxVelocity,Drivetrain.kMaxAcceleration)); // max vel & accel

      // Pathplanner sets 0,0 as the lower left hand corner (FRC field coord system) 
      // for Gazebo, need to subtract intitial pose from each state so that 0,0 is 
      // in the center of the robot 
      Pose2d p0;
     
      p0=trajectory.getInitialHolonomicPose();
      System.out.println("Path started:"+file+" relative to first state");
     
      System.out.println("initial path pose:"+trajectory.getInitialHolonomicPose());
      System.out.println("initial robot pose:"+TargetMgr.startPose());
     
      List<State> states = trajectory.getStates();
      for(int i=0;i<states.size();i++){
        PathPlannerState state=trajectory.getState(i);
        Pose2d p=state.poseMeters;

        Rotation2d h=state.holonomicRotation;
       
        Pose2d pr=p.relativeTo(p0);
        if(i<2)
          pr=new Pose2d(pr.getTranslation(),new Rotation2d());
        if(TargetMgr.getAlliance()==TargetMgr.BLUE)
          state.holonomicRotation=h.plus(new Rotation2d(Math.toRadians(180)));
        if(debug_path && (i%10)==0)
          System.out.format("%d p X:%-3.1f Y:%-3.1f R:%-3.1f H:%-3.1f pr X:%-3.1f Y:%-3.1f R:%-3.1f H:%-3.1f \n",
            i,p.getX(),p.getY(),p.getRotation().getDegrees(),h.getDegrees(),
            pr.getX(),pr.getY(),pr.getRotation().getDegrees(),state.holonomicRotation.getDegrees()
          );
        state.poseMeters=pr;
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
    double r=((PathPlannerState)state).holonomicRotation.getDegrees();
    double angle=Drivetrain.unwrap(last_heading,r);
   
    pd.d[4] =Math.toRadians(angle);
    pd.d[5] = Math.toRadians(m_drive.getHeading());
    last_heading=angle;

    pathdata.add(pd);
  }
  
}
