// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Trajectories extends SubsystemBase {

  public static int CURVED = 0;
  public static int STRAIGHT = 1;
  public static int HOOKED = 2;

  Trajectory m_trajectory;

  /** Creates a new Trajectories. */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static Trajectory straightPath() {
    return straightPath(0, 0, 10);
  }

  public static Trajectory straightPath(double xstart, double ystart, double distance) {
    Trajectory traj = TrajectoryGenerator.generateTrajectory(new Pose2d(xstart, ystart, new Rotation2d(0.0)), List.of(),
        new Pose2d(xstart + distance, ystart, new Rotation2d(0.0)),
        new TrajectoryConfig(Drivetrain.kMaxVelocity, Drivetrain.kMaxAcceleration));
    return traj;
  }

  public static Trajectory hookPath() {
    double y = i2M(26);
    double x = i2M(140);
    return hookPath(x - 2 * y, 0, x, y);
  }

  public static Trajectory hookPath(double xmid, double ymid, double xend, double yend) {
    Trajectory traj = TrajectoryGenerator.generateTrajectory
    (
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(new Translation2d(xmid, ymid)), 
      new Pose2d(xend, yend, new Rotation2d(90.0)),
      new TrajectoryConfig(Drivetrain.kMaxVelocity, Drivetrain.kMaxAcceleration)
    );
    return traj;
  }

  // default (2,2) (8,6)
  public static Trajectory curvedPath() {
    return curvedPath(0, 0, 12, 4);
  }

  public static Trajectory curvedPath(double xstart, double ystart, double xend, double yend) {
    Trajectory traj = TrajectoryGenerator.generateTrajectory(new Pose2d(xstart, ystart, new Rotation2d(0)), List.of(),
        new Pose2d(xend, yend, new Rotation2d(0)),
        new TrajectoryConfig(Drivetrain.kMaxVelocity, Drivetrain.kMaxAcceleration));
    return traj;
  }

  public static double unwrap(double previous_angle, double new_angle) {
    double d = new_angle - previous_angle;
    d = d >= 180 ? d - 360 : (d <= -180 ? d + 360 : d);
    return previous_angle + d;
  }

  public static double i2M(double inches) {
    return inches * 0.0254;
  }
}
