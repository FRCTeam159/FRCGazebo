package gazebo;

import edu.wpi.first.math.controller.PIDController;

// convenience class that combines an encoder and a motor (e.g. SparkMax)
public class SimEncMotor {
    SimMotor motor;
    SimEncoder encoder;
    double setval=0;
    boolean enabled=false;
    double target;
    int id;
    final PIDController pid=new PIDController(5,0.0,0.0);
    public SimEncMotor(int id){
        motor=new SimMotor(id);
        encoder=new SimEncoder(id);
        pid.setTolerance(0.001,0.001);
        this.id=id;
    }
    public void setInverted(){
        encoder.setInverted();
        motor.setInverted();
    }
    public void enable(){
        enabled=true;
        motor.enable();
        encoder.enable();
    }
    public void disable(){
        enabled=false;
        motor.disable();
        encoder.disable();
    }
    public void reset(){
        motor.reset();
        encoder.reset();
    }
    public void set(double v){
        setval=v;
        motor.set(v);
    }
    public double get(){
        return setval;
    }
    public void setDistancePerRotation(double d){
        motor.setDistancePerRotation(d);
        encoder.setDistancePerRotation(d);
    }
    public double getDistancePerRotation() {
        return encoder.getDistancePerRotation();
    }
    public double getDistance() {
        return encoder.getDistance();
    }
    public double getRate() {
        return encoder.getRate();
    }
    public void setPosition(double d){
        target=d;
        pid.setSetpoint(d);
    }
    public void setPosition(){
        double a=encoder.getAbsPosition();
        double err=10*pid.calculate(a);
        //if(id==2)
        //    System.out.println(id+" a:"+a+" err:"+err);
        motor.setAbs(err);
    }
    public boolean atTarget(){
       return pid.atSetpoint();
    }
}
