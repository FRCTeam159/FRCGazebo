// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;


import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import objects.PlotServer;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Drivetrain;
import utils.PathData;
import utils.PlotUtils;

// =================================================
// DrivePath: class constructor (called from RobotContainer)
// =================================================
public class DrivePath extends Command {

  /** Creates a new AutoTest. */
  private ArrayList<PathData> pathdata = new ArrayList<PathData>();

  public boolean holotest=true;
 
  private final HolonomicDriveController m_hcontroller=new HolonomicDriveController(new PIDController(1, 0, 0), new PIDController(1, 0, 0),
  new ProfiledPIDController(3, 0, 0,
    new TrapezoidProfile.Constraints(4*6.28, 4*3.14)));

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
  boolean reversed = true;

  double maxV;
  double maxA;
  double last_time;

  int plot_type = utils.PlotUtils.PLOT_NONE;
  int trajectory_option = Autonomous.PROGRAM;

  public DrivePath(Drivetrain drive, int opt) {
    reversed=false;
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
    
    if(xPath<0)
      reversed=true;

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

    ChassisSpeeds speeds;
   
    speeds= m_hcontroller.calculate(m_drive.getPose(), reference,reference.poseMeters.getRotation());
    
    m_drive.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false);
  
    if(plot_type == PlotUtils.PLOT_LOCATION)
      plotLocation((Trajectory.State)reference);
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
      default:
        System.out.println("ERROR unknown trajectory type:"+trajectory_option);
    }
    return null;
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
    
    pd.d[4] = 2*state.poseMeters.getRotation().getRadians();
    pd.d[5] = 2*current_pose.getRotation().getRadians();

    pathdata.add(pd);
  }
}
