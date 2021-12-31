// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Trajectories;
import frc.robot.subsystems.Simulation;
import utils.PathData;
import utils.PlotUtils;

public class DrivePath extends CommandBase {
  /** Creates a new AutoTest. */
  ArrayList<PathData> pathdata = new ArrayList<PathData>();
  private final RamseteController m_ramsete = new RamseteController();
  private final Timer m_timer = new Timer();
  private final Drivetrain m_drive;
  private final Simulation m_simulation;
  static public boolean plot_trajectory_motion = false;
  static public boolean plot_trajectory_dynamics = false;
  static public boolean plot_dynamics = false;
  static public boolean print_path=false;

  static public boolean publish_path = false;
  static public boolean first_call = true;

  static int data_count = 0;
  double last_heading = 0;
  Trajectory m_trajectory;
  Pose2d initial_pose;
  Trajectory.State initial_state;
  PathData last_data;
  PathData data_sum;
  double runtime;
  double elapsed=0;
  int states;
  int intervals;

  public DrivePath(Drivetrain drive, int type) {
    m_drive = drive;
    m_simulation=m_drive.simulation;
    addRequirements(drive);
    if(type==Trajectories.CURVED)
      setTrajectory(Trajectories.curvedPath());
    else if(type==Trajectories.STRAIGHT)
      setTrajectory(Trajectories.straightPath());
      System.out.println("DrivePath Error: eunknown trajectory option:"+type);
    }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.resetOdometry(m_trajectory.getInitialPose());
    m_timer.reset();
    m_timer.start();
    m_simulation.reset();
    m_simulation.start();

    data_count = 0;
    pathdata.clear();
    System.out.println("runtime:"+runtime+" states:"+states+" intervals:"+intervals);
  }

  // Called every time the scheduler runs while the command is scheduled.\n
  @Override
  public void execute() {
    //elapsed = m_timer.get();
    elapsed=m_simulation.getTime();
    //System.out.println(elapsed+" "+elapsed/runtime);
    Trajectory.State reference = m_trajectory.sample(elapsed);
    //System.out.format("%d %2.3f %2.3f %2.3f\n",data_count,elapsed,reference.timeSeconds,reference.poseMeters.getRotation().getDegrees());
    ChassisSpeeds speeds = m_ramsete.calculate(m_drive.getPose(), reference);
    m_drive.odometryDrive(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
    if (PlotUtils.plot_option==PlotUtils.PLOT_PATH)
      plotPath(reference);
    data_count++;
  }
  public static double d2R(double t){
    return t*2*Math.PI/360.0;  
  }
  // =================================================
  // collect DathData objects for motion error plot (pd)
  // =================================================
  // pd.tm time of sample
  // pd.d[0] expected left distance
  // pd.d[1] observed left distance
  // pd.d[2] expected right distance
  // pd.d[3] observed right distance
  // pd.d[4] expected heading
  // pd.d[5] observed heading
  private void plotPath(Trajectory.State state) {
    PathData cd = PlotUtils.getPathMotion(data_count, state, Drivetrain.kTrackWidth);
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

    double ch=state.poseMeters.getRotation().getDegrees();
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
    System.out.format("%d %2.3f %2.3f %2.3f %2.3f %2.3f %2.3f %2.3f\n",
     data_count, pd.tm, ld, cl, rd, cr, gh, ch);
     //System.out.println(data_count+" "+ pd.tm+" "+ld+" "+cl+" "+rd+" "+cr+" "+gh+" "+ch);

    last_heading = gh;
    last_data = cd;

    pathdata.add(pd);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(m_trajectory==null)
      return;
    System.out.println("Simtime="+m_simulation.getTime()+" Realtime="+runtime);
    m_simulation.end();
    if(PlotUtils.plot_option==PlotUtils.PLOT_PATH)
      PlotUtils.plotPathMotion(pathdata);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elapsed>=runtime || m_trajectory==null;
  }
  public void setTrajectory(Trajectory t) {
    m_trajectory = t;

    if (first_call)
      if (plot_trajectory_dynamics || plot_trajectory_motion) {
        List<Trajectory.State> list = m_trajectory.getStates();
        if (plot_trajectory_motion)
          PlotUtils.plotPathMotion(list, Drivetrain.kTrackWidth);
        if (plot_trajectory_dynamics)
          PlotUtils.plotPathDynamics(list);
      }
    initial_pose = m_trajectory.getInitialPose();
    m_drive.reset();
    initial_state = t.sample(0);
    last_data = PlotUtils.getPathMotion(data_count, initial_state, Drivetrain.kTrackWidth);
    data_sum = new PathData();
    first_call = false;
    runtime=m_trajectory.getTotalTimeSeconds();
    states=m_trajectory.getStates().size();
    intervals=(int)(runtime/0.02);
  }
}
