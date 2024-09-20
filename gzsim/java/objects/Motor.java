// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package objects;
import gazebo.SimEncMotor;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;

/** Add your docs here. */
public class Motor {
    SimEncMotor sim_motor=null;
    CANSparkMax rev_motor=null;
    RelativeEncoder rev_encoder;
    int m_chnl;
    boolean m_inverted=false;
    boolean m_enabled=true;

    double m_dpr=1;
    static boolean m_real=false;
    public Motor(int id) {
        rev_motor = new CANSparkMax(id, CANSparkLowLevel.MotorType.kBrushless);
        rev_encoder = rev_motor.getEncoder();
        sim_motor = new SimEncMotor(id);
        m_chnl=id;
    }
    static public void setMode(boolean m){
        m_real=m;
    }
    public void enable(){
        m_enabled=true;
        if(!m_real)
            sim_motor.enable();
    }
    public void disable(){
        m_enabled=false;
        if(m_real)
            rev_motor.disable();
        else
            sim_motor.disable();
    }
    public double getPosition() {
        if(m_real)
            return rev_encoder.getPosition();
        else
            return sim_motor.getDistance();
    }
    public void setPosition(double d) {
        if(m_real)
            rev_encoder.setPosition(d);
      }
     public double getRotations(){
        return getPosition()/m_dpr;
    }
     public void reset(){
        if(m_real)
            rev_encoder.setPosition(0.0);
        else
            sim_motor.reset();
    }
    public double getVelocity() {
        if(m_real)
            return rev_encoder.getVelocity()/60; // rpm to rps
        else
            return sim_motor.getRate();
    }
    public void setInverted() {
        if(m_real)
            rev_motor.setInverted(true);
        else
            sim_motor.setInverted();
    }
    public void set(double speed) {
        if(m_real)
            rev_motor.set(speed);
        else
            sim_motor.set(speed);
    }
    public void setVoltage(double v) {
        if(m_real)
            rev_motor.setVoltage(v);
        else
            sim_motor.set(v);
    }
    public void setDistancePerRotation(double d){
        m_dpr=d;
        if(m_real)
            rev_encoder.setPositionConversionFactor(d);
        else
            sim_motor.setDistancePerRotation(d);
    }
}
