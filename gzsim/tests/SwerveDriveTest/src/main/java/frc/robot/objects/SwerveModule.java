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
import frc.robot.subsystems.Drivetrain;
import gazebo.SimEncMotor;

public class SwerveModule {
  private static final double kWheelRadius = 0.0508; // 2 inches

  private double distancePerRotation=2*kWheelRadius*Math.PI;

  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration = Drivetrain.kMaxAngularAcceleration;

  private SimEncMotor m_driveMotor;
	private SimEncMotor m_turnMotor;

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(0.1, 0, 0);
  //private final PIDController m_turningPIDController = new PIDController(0.2, 0, 0);

  private final PIDController simpleTurnPIDController=new PIDController(1, 0, 0);

  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(1,0,0,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
 // private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
 // private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);
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
		m_turnMotor.setDistancePerRotation(360.0);
    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    //m_driveEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
   // m_turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
     //m_turningPIDController.enableContinuousInput(-180, 180);
     simpleTurnPIDController.enableContinuousInput(-Math.PI, Math.PI);
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
    double angle=m_turnMotor.getDistance();
    //angle=angle%360.0;
    angle=Math.toRadians(angle);

    //SwerveModuleState state = new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);

     SwerveModuleState state =SwerveModuleState.optimize(desiredState, new Rotation2d(angle));
    //SwerveModuleState state =SwerveModuleState.optimize(desiredState, new Rotation2d(0));

    //var delta = desiredState.angle.minus(new Rotation2d(angle));
    //System.out.println(delta);
    // Calculate the drive output from the drive PID controller.
    final double driveOutput = m_drivePIDController.calculate(m_driveMotor.getRate(), state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    //final double turnOutput = simpleTurnPIDController.calculate(angle,state.angle.getDegrees());
    //System.out.println(m_drive_chnl/2+" "+angle+" "+state.angle.getDegrees()+" "+(-state.speedMetersPerSecond>0?"+":"-")+" "+turnOutput);

    double turnOutput = simpleTurnPIDController.calculate(angle,state.angle.getRadians());
    //setAngle(state.angle.getDegrees(),driveOutput + driveFeedforward);

    m_driveMotor.set(driveOutput + driveFeedforward);

    final double turnFeedforward =m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    m_turnMotor.set(turnOutput); 

    //m_driveMotor.set(driveOutput + driveFeedforward);
   // m_turnMotor.set(turnOutput + turnFeedforward); 
    //m_turnMotor.set(turnOutput); 

    }

// 0 45.0 0.3323947409004224 22.33380262954979 1.0
// 1 -45.0 0.00563591450841582 -22.50281795725421 -1.0
// 2 -45.0 -0.055179341753442204 -22.472410329123278 1.0
// 3 45.0 0.41032534622191336 22.294837326889045 -1.0
    
   public void turn(double value){
    m_turnMotor.set(value);
  }
  public void setAngle(double angle, double d){
    double current=m_turnMotor.getDistance();
    current=current%360;
    double turnOutput = simpleTurnPIDController.calculate(current,angle );
    //System.out.println(m_drive_chnl/2+" "+angle+" "+current+" "+turnOutput+" "+d);
    m_driveMotor.set(d);
    m_turnMotor.set(turnOutput); 
  }
  public void move(double value){
    m_driveMotor.set(value);
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
