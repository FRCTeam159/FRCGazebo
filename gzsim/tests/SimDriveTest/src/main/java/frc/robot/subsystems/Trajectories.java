// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Trajectories extends SubsystemBase {

public static double MAX_VEL = 1.7;
public static double MAX_ACC = 2.6;
public static double MAX_JRK = 1.3;
public static double KP = 4.0;
public static double KD = 0.0;
public static double KI = 0.0;
public static double GFACT = 10.0;
public static double KV = 1.0 / MAX_VEL;
public static double KA = 0.0;

static final public int STRAIGHT=0;
static final public int SCURVE=1;

  Trajectory m_trajectory;
  /** Creates a new Trajectories. */
  public Trajectories() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public static Trajectory straightPath(){
    return straightPath(2,2,10);
  }
  public static Trajectory straightPath(double xstart, double ystart, double distance){
    Trajectory traj = TrajectoryGenerator.generateTrajectory(
      new Pose2d(xstart, ystart, new Rotation2d()),
      List.of(),
      new Pose2d(xstart+distance, ystart+distance, new Rotation2d()),
      new TrajectoryConfig(MAX_VEL, MAX_ACC)
      );
      return traj;
  }
  // default (2,2) (8,6)
  public static Trajectory curvedPath(){
    return curvedPath(2,2,8,6);
  }
  public static Trajectory curvedPath(double xstart, double ystart, double xend, double yend){
    Trajectory traj = TrajectoryGenerator.generateTrajectory(
      new Pose2d(xstart, ystart, new Rotation2d()),
      List.of(),
      new Pose2d(xend, yend, new Rotation2d()),
      new TrajectoryConfig(MAX_VEL, MAX_ACC)
      );
      return traj;
  }
}
