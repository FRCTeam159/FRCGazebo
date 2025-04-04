package gazebo;


// convenience class that combines an encoder and a motor (e.g. SparkMax)
public class SimEncMotor {
    SimMotor motor;
    SimEncoder encoder;
    double setval=0;
    boolean enabled=false;
    double target;
    int id;
    boolean inverted=false;
    public SimEncMotor(int id){
        motor=new SimMotor(id);
        encoder=new SimEncoder(id);
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
    
}
