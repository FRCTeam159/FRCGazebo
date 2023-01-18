// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.objects;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.DriveTrain;
import gazebo.SimEncMotor;

public class SwerveModule {
  private static final double kWheelRadius = Units.inchesToMeters(2.0); // 2 inches

  private double gazebo_scale=1;  // not sure why but grid marks in gazebo are somewhat larger than a meter ?
  private double distancePerRotation=gazebo_scale*2*kWheelRadius*Math.PI;

  private static final double kModuleMaxAngularVelocity = DriveTrain.kMaxAngularSpeed; //degrees per second;
  private static final double kModuleMaxAngularAcceleration = DriveTrain.kMaxAngularAcceleration;// degrees per second per second

  private SimEncMotor m_driveMotor;
	private SimEncMotor m_turnMotor;

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(10, 0.0, 0);
  //private final PIDController m_turningPIDController = new PIDController(3, 0.05, 0);
  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(5,0.0,0,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0.1, 0.1);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(0.1, 0.1);

  public int m_drive_chnl;
  public int m_turn_chnl;
  
  boolean m_enabled=false;
  int cnt=0;
  static boolean debug=false;
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
    m_enabled=true;
    m_driveMotor.enable();
    m_turnMotor.enable();
  }
  public void disable(){
   // m_turningPIDController.reset(0.0);

    m_enabled=false;
    m_driveMotor.disable();
    m_turnMotor.disable();
  }
  public void reset(){
    //m_turningPIDController.reset(0.0);
    m_drivePIDController.reset();
    m_turningPIDController.reset(0.0);
    m_enabled=false;
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

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(m_driveMotor.getDistance(), new Rotation2d(Math.toRadians(m_turnMotor.getDistance())));
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

     SwerveModuleState state =desiredState;
     if(m_enabled)
      state=SwerveModuleState.optimize(desiredState, new Rotation2d(angle));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput = m_drivePIDController.calculate(m_driveMotor.getRate(), state.speedMetersPerSecond);
    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    //System.out.println(m_drive_chnl/2+" "+angle+" "+state.angle.getDegrees()+" "+(-state.speedMetersPerSecond>0?"+":"-")+" "+turnOutput);

    final double turnOutput = m_turningPIDController.calculate(angle,state.angle.getRadians());
    final double turnFeedforward = 0;//m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);
    //final double turnFeedforward = m_turnFeedforward.calculate(state.angle.getRadians());

    //System.out.format("%d drive:%-4f out:%-4.1f ff:%-4.1f turn angle:%-4.1f rate:%-4.1f out:%-4.2f ff:%-3.2f\n",
    //     m_drive_chnl/2,m_driveMotor.getRate(),driveOutput,driveFeedforward,angle,turn_rate,turnOutput,turnFeedforward);
     if((debug && (cnt%100)==0)){
     System.out.format("%d drive:%-4.2f state:%-4.2f out:%-4.2f\n",
          m_drive_chnl/2,m_driveMotor.getRate(),state.speedMetersPerSecond,driveOutput);
     System.out.format("%d turn:%-4.2f state:%-4.2f out:%-4.2f\n",
          m_drive_chnl/2,angle,state.angle.getRadians(),turnOutput);
     }
    //if(m_enabled){
      //m_turnMotor.set(turnOutput); 
      //m_driveMotor.set(driveOutput);
      m_turnMotor.set(turnOutput + turnFeedforward); 
      m_driveMotor.set(driveOutput + driveFeedforward);
   // }
   cnt++;

  }

  // just apply a voltage to the turn motor
  public void turn(double value){
    m_turnMotor.set(value);
  }

  public double getXVoltage(){
    return  m_driveMotor.get();
  }
  public double getYVoltage(){
    return  m_turnMotor.get();
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
