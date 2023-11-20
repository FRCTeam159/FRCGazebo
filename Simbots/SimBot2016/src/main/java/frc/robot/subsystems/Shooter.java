// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.ExecShooter;
import gazebo.SimEncMotor;
import gazebo.SimGyro;
import gazebo.SimSwitch;

public class Shooter extends SubsystemBase implements RobotMap {

    static final double FWSPEED = 75;
    static final double FP = 0.01;
    static final double FI = 0.02;
    static final double FD = 0.0;

    static final double AP = 0.05;
    static final double AI = 0.002;
    static final double AD = 0.0001;

    static final double AMIN = 0;
    static final double AMAX = 70;
    static final double MAX_SPEED_ERROR = 10;
    static final double MAX_SPEED_DELTA_ERROR = 1;

    static final double MAX_ANGLE_ERROR = 2;
    static final double GOTO_LOWER_SPEED = -0.6;
    static final double PID_UPDATE_PERIOD = 0.01;

    double target_angle, max_angle, min_angle;
    double max_angle_error;
    double flywheel_target;
    double flywheel_speed;
    boolean flywheels_enabled;

    boolean initialized;

    SimEncMotor angleMotor = new SimEncMotor(SHOOTER_MOTOR);
    SimEncMotor flywheelMotor = new SimEncMotor(FLYWHEEL_MOTOR);

    PIDController angle_pid;
    PIDController flywheel_pid;

    SimGyro angle_gyro = new SimGyro(SHOOTER_GYRO,SimGyro.Mode.PITCH);
    SimSwitch angle_limit = new SimSwitch(SHOOTER_SWITCH);

    /** Creates a new Shooter. */
    public Shooter() {
        max_angle = AMAX; // max elevation (degrees)
        min_angle = AMIN;
        flywheel_target = FWSPEED;
        flywheel_speed = 0;
        max_angle_error = MAX_ANGLE_ERROR;
        target_angle = 0;
        angle_pid = new PIDController(AP, AI, AD);
        angle_pid.setSetpoint(0);
        angle_pid.setTolerance(MAX_ANGLE_ERROR);

        flywheel_pid = new PIDController(FP, FI, FD);
        flywheel_pid.setSetpoint(0);
        flywheel_pid.setTolerance(MAX_SPEED_ERROR,MAX_SPEED_DELTA_ERROR);
        initialized = false;

        angleMotor.enable();
        flywheelMotor.enable();
        angle_gyro.enable();
        angle_gyro.reset();
    }

    public void initDefaultCommand() {
        setDefaultCommand(new ExecShooter());
    }
    // Initialize
    public void init() {
        initialized = true;
        angle_pid.reset();
        angle_gyro.reset();
        flywheelMotor.enable();
        enableFlywheels();
        //angleMotor.disable();
        //flywheelMotor.reset();
    }

    public void reset() {
        disableFlywheels();
        setTargetAngle(0);
        initialized = false;
        angle_pid.reset();
    }

    public void disable() {
        angle_pid.reset();
        // angleMotor.disable();
        // flywheelMotor.disable();
        // angleMotor.disable();
    }

    public void log() {
        SmartDashboard.putNumber("FW Speed", getFWSpeed());
        SmartDashboard.putNumber("Set FW Speed",flywheel_target);
        SmartDashboard.putNumber("Shooter Angle", getShooterAngle());
        SmartDashboard.putNumber("Shooter Set Angle",getTargetAngle());
    }

    public double getFWSpeed() {
        return flywheelMotor.getRate();
    }

    public double getShooterAngle() {
        double d =(angle_gyro.getAngle());
        return d;
    }

    public double getTargetAngle(){
        return target_angle;
    }
    public double getTargetSpeed(){
        return flywheel_target;
    }

    public boolean isAtAngle(){
        return angle_pid.atSetpoint();
    }
    public boolean isAtSpeed(){
        double ave_speed=getFWSpeed();
        boolean ontarget=Math.abs(ave_speed-flywheel_target)<=MAX_SPEED_ERROR;
        return ontarget;
    }
    // Set the shooter angle
    public void setTargetAngle(double a) {
        a = a > max_angle ? max_angle : a;
        a = a < min_angle ? min_angle : a;
        target_angle = a;
        angle_pid.setSetpoint(target_angle);
        angle_pid.setTolerance(max_angle_error);
        //System.out.println("angle="+angle);
    }

    public boolean flywheelsEnabled(){
        return flywheels_enabled;
    }
    public void enableFlywheels(){
        flywheel_pid.reset();
        //flywheelMotor.reset();
        flywheel_target=FWSPEED;
        flywheels_enabled=true;
        //flywheelMotor.set(flywheel_target);
        //flywheelMotor.enable();

    }
    public void disableFlywheels(){
        flywheels_enabled=false;
        flywheel_pid.reset();
        flywheelMotor.set(0);
        flywheel_target=0;
        //flywheelMotor.disable();
        //flywheelMotor.reset();
    }

    void setInitialized() {
        initialized=true;
        //angleMotor.setSetpoint(0.0);
        target_angle=0;
        angle_gyro.reset();
        angleMotor.enable();
    }
    
    void initialize() {
        if(!atLowerLimit()){
            //angleMotor.disable();
            goToLowerLimitSwitch();
        }
    }
    boolean isInitialized() {
        return initialized;
    }
    
    void goToLowerLimitSwitch() {
        angleMotor.set(GOTO_LOWER_SPEED);
    }
    
    boolean atLowerLimit() {
        return angle_limit.lowLimit();
    }
    
    void setMaxAngleError(double d) {
        max_angle_error=d;
    }
    public double getMaxAngle() { return max_angle;}
	public double getMinAngle() { return min_angle;}
    public void adjustAngle(){
        double angle=getShooterAngle();
        double err=angle_pid.calculate(angle);
        angleMotor.set(err);
    }
    public void adjustSpeed(){
        double speed=getFWSpeed();
        double err=flywheel_pid.calculate(speed,flywheel_target);
        flywheelMotor.set(err);
    }
    @Override
    public void periodic() {
        //if(Robot.mode==Robot.Mode.SHOOTING){
        adjustAngle();
        adjustSpeed();
        //}
        log();
    }
}
