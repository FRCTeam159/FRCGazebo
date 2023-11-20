package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.ExecLoader;
import gazebo.SimEncMotor;
import gazebo.SimGyro;
import gazebo.SimMotor;
import gazebo.SimSwitch;

public class Loader extends SubsystemBase implements RobotMap {
    public static final double LOAD_ROLLER_SPEED = 0.9;
    public static final double SETZEROSPEED = -0.2;
    public static final double LIFT_ASSIST_SPEED = 0.1;
    public static final double PID_UPDATE_PERIOD = 0.0;
    
    public static final double LOAD_PICKUP_ANGLE = 10;

    public static final double MED_ANGLE = 6;
    public static final double HIGH_ANGLE = 12;
    public static final double LOW_ANGLE = 0.0;
    public static final double MAX_ANGLE = 50;

    public static final double MAX_ANGLE_ERROR =1;
    public static final double P =0.1;
    public static final double I =0.0002;
    public static final double D =0.0;

    public static final int ROLLERS_OFF =0;
    public static final int ROLLERS_FORWARD =1;
    public static final int ROLLERS_REVERSE =2;

    double target_angle=0;
	double max_angle=80;
	double min_angle=0;
	double roller_speed=0;

    int roller_state=ROLLERS_OFF;
	boolean initialized=false;
	boolean loading=false;
    
    SimEncMotor liftMotor = new SimEncMotor(LOADER_ARMS_MOTOR);
	SimMotor rollerMotor = new SimMotor(ROLLER_MOTOR);
	PIDController angle_pid;
	SimGyro angle_gyro = new SimGyro(LOADER_GYRO,SimGyro.Mode.PITCH);
    SimSwitch lifterLowerLimit = new SimSwitch(LOADER_ARMS_SWITCH);

    public Loader(){
        angle_pid=new PIDController(P, I, D);
        angle_pid.setTolerance(MAX_ANGLE_ERROR);
        angle_gyro.reset();
        lifterLowerLimit.enable();
        liftMotor.enable();
        rollerMotor.enable();
        angle_gyro.enable();
    }
    public void initDefaultCommand() {
        setDefaultCommand(new ExecLoader());
    }
    public void log() {
        SmartDashboard.putBoolean("Loading", loading());
        SmartDashboard.putNumber("Lifter Angle", -angle_gyro.getAngle());
        SmartDashboard.putNumber("Lifter Set Angle", target_angle);

        SmartDashboard.putNumber("Rollers", roller_state);
    }
    
    public void setLow() {
        setRollerState(ROLLERS_OFF);
        //goToZeroLimitSwitch();
        setLifterAngle(LOW_ANGLE);
        loading=false;
    }

    public void loadBall() {
       // liftMotor.disable();
        //liftMotor.set(LIFT_ASSIST_SPEED);
        roller_speed=LOAD_ROLLER_SPEED;
        setLifterAngle(LOAD_PICKUP_ANGLE);
        loading=true;
    }

    public boolean loading() {
        return loading;
    }
    public void setLoading(boolean b) {
        loading=b;
    }
    // Initialize
    public void init(){
        initialized=true;
        angle_pid.reset();
        roller_speed=LOAD_ROLLER_SPEED;
        angle_gyro.reset();
        //log();
    }
    // public void disable(){
    //     angle_pid.reset();
    //     liftMotor.reset();
    //     liftMotor.disable();
    //     rollerMotor.disable();
    //     initialized=false;
    //     roller_speed=0;
    //     loading=false;
    // }
   
    public void setLifterAngle(double a){
        a=a>max_angle?max_angle:a;
        a=a<min_angle?min_angle:a;
        target_angle=a;
        angle_pid.setSetpoint(target_angle);
        angle_pid.setTolerance(MAX_ANGLE_ERROR);
        //liftMotor.enable();
        //log();
    }

    public double getTargetAngle(){
        return target_angle;
    }
    public void spinRollers() {
        switch(roller_state){
        case ROLLERS_OFF:
            rollerMotor.set(0.0);
            break;
        case ROLLERS_FORWARD:
            rollerMotor.set(roller_speed);
            break;
        case ROLLERS_REVERSE:
            rollerMotor.set(-roller_speed);
            break;
        }
    }
    public void setRollerState(int b) {
        roller_state=b;
    }

    public int getRollorState() { 
        return roller_state;
    }
    public void execLoad() {
        //liftMotor.set(LIFT_ASSIST_SPEED);
        setLifterAngle(LOAD_PICKUP_ANGLE);
        setRollerState(ROLLERS_FORWARD);
    }

    public boolean lifterAtLowerLimit() {
        return lifterLowerLimit.lowLimit();
    }
    public void goToZeroLimitSwitch() {
        if(!lifterAtLowerLimit())
            liftMotor.set(SETZEROSPEED);
        else
            liftMotor.set(0.0);
    }
    // public void setInitialized() {
    //     initialized=true;
    //     liftMotor.set(0.0);
    //     angle_gyro.reset();
    // }
    // public void initialize() {
    //     if(!lifterAtLowerLimit()){
    //         liftMotor.disable();
    //         goToZeroLimitSwitch();
    //     }
    // }
    // public boolean isInitialized() {
    //     return initialized;
    // }
    public void reset() {
        setLifterAngle(0);
        setRollerState(ROLLERS_OFF);
        angle_pid.reset();
    }

    public double getLifterAngle() {
        double d =-(angle_gyro.getAngle());
        return d;
    }
    @Override
    public void periodic() {
        spinRollers();
       // if(Robot.mode==Robot.Mode.LOADING){
        double angle=getLifterAngle();
        double cor=angle_pid.calculate(angle,target_angle);
        //System.out.println("angle:"+angle+" cor:"+cor);
        
        liftMotor.set(cor);
        //}
        log();
    }
    
}
