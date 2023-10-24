/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.objects;

import edu.wpi.first.wpilibj.Timer;
import gazebo.SimEncMotor;
import utils.Averager;

/**
 * Add your docs here.
 */
public class SparkMotor implements MotorInterface {
    // private RelativeEncoder encoder;
    private double zeroValue = 0;
    private double distancePerRotation = 1;
    private int inverted = 1;
    private SimEncMotor sim_motor;
   // private CANSparkMax real_motor;
    private double scale=1;
    private double last_velocity=0;
    private double last_time=0;
    private Averager acc_averager = new Averager(10);
    private Averager vel_averager = new Averager(3);
    private double ave_acc = 0;
    private double ave_vel = 0;
    private double max_acc=-1;
    private int chnl;

    private Timer timer=new Timer();

    public SparkMotor(int id) {
        chnl=id;
        // if (Robot.isReal()) {
        //     real_motor = new CANSparkMax(id, CANSparkMaxLowLevel.MotorType.kBrushless);
        //     zeroValue = real_motor.getEncoder().getPosition();
        // } else {
            sim_motor = new SimEncMotor(id);
        //}
        timer.start();
        enable();
        // System.out.println("IsReal "+Robot.isReal());
    }

    public void setScale(double f){
        scale=f;
    }
    //private double getRotations() {
        //return real_motor.getEncoder().getPosition();
        //return real_motor.getEncoder().getPosition();

    //}
    public double getAcceleration() {
        double tm=timer.get();
        double vel=getRate();
        double accel=0;
        if(tm>0){
            accel=(vel-last_velocity)/(tm-last_time);
        }
        last_velocity=vel;
        last_time=tm; 
        return accel; 
    }
    public void setMaxAccel(double a){
        max_acc=a;
    }
    public double aveVelocity(){
        return ave_vel;
    }
    public double aveAcceleration(){
        return ave_acc;
    }
    @Override
    public void reset() {
        // if (Robot.isReal())
        //     zeroValue = real_motor.getEncoder().getPosition();
        // else
            sim_motor.reset();
    }

    @Override
    public double getRate() {
        // if (Robot.isReal())
        //     return distancePerRotation * inverted * real_motor.getEncoder().getVelocity() / 60;
        // else
            return distancePerRotation * sim_motor.getRate();
    }

    @Override
    public void setDistancePerRotation(double d) {
        distancePerRotation = d;
        sim_motor.setDistancePerRotation(d);
    }

    @Override
    public double getDistance() {
        // if (Robot.isReal())
        //     return (getRotations()-zeroValue) * distancePerRotation * inverted;
        // else
            return sim_motor.getDistance();
    }
    
    @Override
    public void setInverted() {
        inverted = -1;
        sim_motor.setInverted();
    }
    @Override
    public void enable() {
       // if (!Robot.isReal())
            sim_motor.enable();
        timer.reset();
    }

    @Override
    public void set(double v) {
        double r=scale*v/distancePerRotation;
        double mag_acc=Math.abs(ave_acc);  
        ave_acc = acc_averager.getAve(getAcceleration());
        ave_vel = vel_averager.getAve(getRate());
        if(max_acc>0 && mag_acc>max_acc){
            //System.out.println(chnl+" acc:"+mag_acc+" r1:"+r+" r2:"+r*max_acc/mag_acc);
            r*=max_acc/mag_acc;
        }
        // if (Robot.isReal())
        //     real_motor.set(r);
        // else
            sim_motor.set(r);
    }

    @Override
    public void disable() {
        // if (Robot.isReal())
        //     real_motor.disable();
        // else
            sim_motor.disable();
    }
}
