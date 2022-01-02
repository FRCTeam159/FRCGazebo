// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Simulation;
import frc.robot.subsystems.Trajectories;
import utils.PathData;
import utils.PlotUtils;

public class DrivePath extends CommandBase {
  /** Creates a new AutoTest. */
  ArrayList<PathData> pathdata = new ArrayList<PathData>();
  private final RamseteController m_ramsete = new RamseteController();
  private final Timer m_timer = new Timer();
  private final Drivetrain m_drive;
  //private final Simulation m_simulation;
  static public boolean plot_trajectory_motion = false;
  static public boolean plot_trajectory_dynamics = false;

  private double lastVelocity = 0;
  private double lastTime = 0;

  static public boolean print_path = false;

  static public boolean publish_path = false;
  static public boolean first_call = true;

  utils.Averager acc_average =new utils.Averager(20);
  utils.Averager vel_average =new utils.Averager(10);

  static int data_count = 0;
  static int vel_count = 0;
  double last_heading = 0;
  Trajectory m_trajectory;
  int m_type;
  Pose2d initial_pose;
  Trajectory.State initial_state;
  PathData last_data;
  PathData data_sum;
  double runtime;
  double elapsed = 0;
  int states;
  int intervals;

  public DrivePath(Drivetrain drive, int type) {
    m_drive = drive;
    //m_simulation = m_drive.simulation;
    m_type=type;
    addRequirements(drive);
  
  }
 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_type == Trajectories.CURVED)
      setTrajectory(Trajectories.curvedPath());
    else if (m_type == Trajectories.STRAIGHT)
      setTrajectory(Trajectories.straightPath());
    System.out.println("DrivePath Error: unknown trajectory option:" + m_type);
    // Pose2d tol=new Pose2d(0.1,0.1,new Rotation2d(1.0));
    // m_ramsete.setTolerance(tol);

    m_drive.resetOdometry(m_trajectory.getInitialPose());
    acc_average.reset();
    vel_average.reset();
    m_timer.reset();
    m_timer.start();
    // m_drive.simulation.reset();
    // m_drive.simulation.start();
    lastVelocity = 0;
    data_count = 0;
    vel_count = 0;
    pathdata.clear();
    m_drive.startAuto();
    // m_drive.enable();
    System.out.println("runtime:" + runtime + " states:" + states + " intervals:" + intervals);
  }
 
  // Called every time the scheduler runs while the command is scheduled.\n
  @Override
  public void execute() {
    // elapsed = m_timer.get();
    elapsed = m_drive.getTime();
    if(elapsed<0.02){
      return;
    }

    // System.out.println(elapsed+" "+elapsed/runtime);
    Trajectory.State reference = m_trajectory.sample(elapsed);
   
    ChassisSpeeds speeds = m_ramsete.calculate(m_drive.getPose(), reference);
    m_drive.odometryDrive(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
    if (PlotUtils.auto_plot_option == PlotUtils.PLOT_DISTANCE)
      plotDistance(reference);
    else if (PlotUtils.auto_plot_option == PlotUtils.PLOT_DYNAMICS)
      plotDynamics(reference);
    else if (PlotUtils.auto_plot_option == PlotUtils.PLOT_POSITION)
      plotPosition(reference);
    data_count++;
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (m_trajectory == null)
      return;
    System.out.println("Simtime=" + m_drive.getTime() + " Realtime=" + runtime);
    //m_simulation.end();
    m_drive.disable();
    if (PlotUtils.auto_plot_option == PlotUtils.PLOT_DISTANCE){
      String label_list[] = { "Left Travel", "Target", "Right Travel", "Target","Heading","Target"};
      PlotUtils.genericTimePlot(pathdata,label_list,6);
    }
    else if (PlotUtils.auto_plot_option == PlotUtils.PLOT_DYNAMICS){
      String label_list[] = { "Origin Distance", "Target", "Velocity", "Target","Acceleration","Target"};
      PlotUtils.genericTimePlot(pathdata,label_list,6);
    }
    else if (PlotUtils.auto_plot_option == PlotUtils.PLOT_POSITION){
      String label_list[] = { "Left Wheels", "Target", "Center", "Target","Right Wheels","Target"};
      PlotUtils.genericXYPlot(pathdata,label_list,6);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elapsed >= 1.01*runtime || m_trajectory == null;
  }

  public static double d2R(double t) {
    return t * 2 * Math.PI / 360.0;
  }

  public void setTrajectory(Trajectory t) {
    m_trajectory = t;

    initial_pose = m_trajectory.getInitialPose();
    //m_drive.reset();
    initial_state = t.sample(0);
    last_data = PlotUtils.getPathPosition(0, initial_state.poseMeters, Drivetrain.kTrackWidth);
    data_sum = new PathData();
    first_call = false;
    runtime = m_trajectory.getTotalTimeSeconds();
    states = m_trajectory.getStates().size();
    intervals = (int) (runtime / 0.02);
  }
  
  // =================================================
  // collect DathData objects for motion error plot
  // - converts position info an xy plot
  // =================================================
  // pd.tm time of sample
  // pd.d[0] expected left y vs x
  // pd.d[1] observed left y vs x
  // pd.d[2] expected center y vs x
  // pd.d[3] observed center y vs x
  // pd.d[4] expected right y vs x
  // pd.d[5] observed right y vs x
  // =================================================
 private void plotPosition(Trajectory.State state) {
  PathData pd = new PathData();
  double tm=state.timeSeconds;
  PathData t_data = PlotUtils.getPathPosition(tm, state.poseMeters, Drivetrain.kTrackWidth);
  PathData d_data = PlotUtils.getPathPosition(tm, m_drive.getPose(), Drivetrain.kTrackWidth);

  pd.d[0]=d_data.d[0];
  pd.d[1]=d_data.d[1];
  pd.d[2]=t_data.d[0];
  pd.d[3]=t_data.d[1];

  pd.d[4]=d_data.d[2];
  pd.d[5]=d_data.d[3];
  pd.d[6]=t_data.d[2];
  pd.d[7]=t_data.d[3];

  pd.d[8]=d_data.d[4];
  pd.d[9]=d_data.d[5];
  pd.d[10]=t_data.d[4];
  pd.d[11]=t_data.d[5];

  pathdata.add(pd);
}
  // =================================================
  // collect DathData objects for motion error plot
  // - converts position info into distance traveled
  // =================================================
  // pd.tm time of sample
  // pd.d[0] expected left distance
  // pd.d[1] observed left distance
  // pd.d[2] expected right distance
  // pd.d[3] observed right distance
  // pd.d[4] expected heading
  // pd.d[5] observed heading
  // =================================================
 private void plotDistance(Trajectory.State state) {
    double tm=state.timeSeconds;
    Pose2d pose=state.poseMeters;
    PathData cd = PlotUtils.getPathPosition(tm,pose, Drivetrain.kTrackWidth);
    PathData delta = cd.minus(last_data);
    data_sum = data_sum.plus(delta); // accumulated travel
    // left wheels
    double lx = data_sum.d[0];
    double ly = data_sum.d[1];
    double cl = Math.sqrt(lx * lx + ly * ly);
    // right wheels
    double rx = data_sum.d[4];
    double ry = data_sum.d[5];
    double cr = Math.sqrt(rx * rx + ry * ry);
    PathData pd = new PathData();

    double ch = state.poseMeters.getRotation().getDegrees();
    ch = ch > 180 ? ch - 360 : ch; // convert to signed angle fixes problem:th 0->360 gh:-180->180
    double ld = m_drive.getLeftDistance();
    double rd = m_drive.getRightDistance();
    double gh = m_drive.getHeading();
    gh = Trajectories.unwrap(last_heading, gh);

    pd.tm = state.timeSeconds;
    pd.d[0] = ld;
    pd.d[1] = cl;
    pd.d[2] = rd;
    pd.d[3] = cr;
    pd.d[4] = d2R(gh);
    pd.d[5] = d2R(ch);

    if (print_path)
      System.out.format("%d %2.3f %2.3f %2.3f %2.3f %2.3f %2.3f %2.3f\n", data_count, pd.tm, ld, cl, rd, cr, gh, ch);
    last_heading = gh;
    last_data = cd;
    pathdata.add(pd);
  }

  // =================================================
  // collect DathData objects for dynamics error plot
  // =================================================
  // pd.tm time of sample 
  // pd.d[0] current distance from start traveled
  // pd.d[1] target distance from start traveled
  // pd.d[2] current velocity
  // pd.d[3] target velocity
  // pd.d[4] current acceleration
  // pd.d[5] target acceleration
  // =================================================
  private void plotDynamics(Trajectory.State state) {
    PathData pd = new PathData();
    
    double dt = elapsed - lastTime;

    double h = m_drive.getHeading();
    h = Trajectories.unwrap(last_heading, h);

    double tm = elapsed;
    double x = m_drive.getPose().getX();
    double y = m_drive.getPose().getY();

    double obs_distance=Math.sqrt(x*x+y*y);
    x=state.poseMeters.getX();
    y=state.poseMeters.getY();
    double exp_distance=Math.sqrt(x*x+y*y);

    double acceleration=0;
    double a=0;
    double v = vel_average.getAve(m_drive.getVelocity());

    if (data_count > vel_average.numAves()){
      acceleration =  (v - lastVelocity) / 0.02;
    }
    
    a = acc_average.getAve(acceleration);
   
    pd.tm = tm;
    pd.d[0] = obs_distance;
    pd.d[1] = exp_distance;
    pd.d[2] = v;
    pd.d[3] = state.velocityMetersPerSecond;
    pd.d[4] = a;
    pd.d[5] = state.accelerationMetersPerSecondSq;
    pathdata.add(pd);
    lastTime = elapsed;
    lastVelocity = v;
  }

  
}
