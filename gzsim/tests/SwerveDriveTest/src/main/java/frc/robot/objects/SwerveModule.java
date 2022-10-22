// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.objects;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import gazebo.SimEncMotor;

public class SwerveModule {
  private static final double kWheelRadius = 0.0508; // 2 inches

  private double distancePerRotation=2*kWheelRadius*Math.PI;

  private static final double kModuleMaxAngularVelocity = Math.toRadians(0); //degrees per second;
  private static final double kModuleMaxAngularAcceleration = Math.toRadians(0);// degrees per second per second

  private SimEncMotor m_driveMotor;
	private SimEncMotor m_turnMotor;

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(0.5, 0, 0);

  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(1,0,0,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 1);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 1);

  public int m_drive_chnl;
  public int m_turn_chnl;

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   */
  public SwerveModule(int driveMotorChannel,int turningMotorChannel) {
    m_drive_chnl=driveMotorChannel;
    m_turn_chnl=turningMotorChannel;

    m_driveMotor=new SimEncMotor(driveMotorChannel);
    m_turnMotor=new SimEncMotor(turningMotorChannel);

    m_driveMotor.setDistancePerRotation(distancePerRotation);
		m_turnMotor.setDistancePerRotation(360.0); // getDistance will return degrees*rotations

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
     m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void enable(){
    m_driveMotor.enable();
    m_turnMotor.enable();
  }
  public void disable(){
    m_driveMotor.disable();
    m_turnMotor.disable();
  }
  public void reset(){
    m_driveMotor.reset();
    m_turnMotor.reset();
  }
  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveMotor.getRate(), new Rotation2d(Math.toRadians(m_turnMotor.getDistance())));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    double angle=m_turnMotor.getDistance(); // rotations in degrees
    angle=angle%360.0;
    angle=Math.toRadians(angle);

     SwerveModuleState state =SwerveModuleState.optimize(desiredState, new Rotation2d(angle));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput = m_drivePIDController.calculate(m_driveMotor.getRate(), state.speedMetersPerSecond);
    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    //System.out.println(m_drive_chnl/2+" "+angle+" "+state.angle.getDegrees()+" "+(-state.speedMetersPerSecond>0?"+":"-")+" "+turnOutput);

    final double turnOutput = m_turningPIDController.calculate(angle,state.angle.getRadians());
    final double turnFeedforward =m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    m_turnMotor.set(turnOutput+turnFeedforward); 
    m_driveMotor.set(driveOutput + driveFeedforward);

  }

  // just apply a voltage to the turn motor
  public void turn(double value){
    m_turnMotor.set(value);
  }

  // just apply a voltage to the wheel motor
  public void move(double value){
    m_driveMotor.set(value);
  }
  // use a PID controller to set an explicit turn angle
  public void setAngle(double a, double d){
    double current=m_turnMotor.getDistance();
    current=current%360;
    double turnOutput = m_turningPIDController.calculate(Math.toRadians(current),Math.toRadians(a) );
    //System.out.println(m_drive_chnl/2+" "+a+" "+current+" "+turnOutput+" "+d);
    m_driveMotor.set(d);
    m_turnMotor.set(turnOutput); 
  }
  
  public double getDistance(){
    return  m_driveMotor.getDistance();
  }
  public double getAngle(){
    return  m_turnMotor.getDistance();
  }
  public double getMoveRate(){
    return  m_driveMotor.getRate();
  }
  public double getTurnRate(){
    return  m_turnMotor.getRate();
  }
  public void setInverted(){
    m_driveMotor.setInverted();
  }
}
