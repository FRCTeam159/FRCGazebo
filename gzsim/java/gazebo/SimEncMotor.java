package gazebo;

// convenience class that combines an encoder and a motor (e.g. SparkMax)
public class SimEncMotor {
    SimMotor motor;
    SimEncoder encoder;
    public SimEncMotor(int id){
        motor=new SimMotor(id);
        encoder=new SimEncoder(id);
    }
    public void setInverted(){
        encoder.setInverted();
        motor.setInverted();
    }
    public void enable(){
        motor.enable();
        encoder.enable();
    }
    public void disable(){
        motor.disable();
        encoder.disable();
    }
    public void reset(){
        motor.reset();
        encoder.reset();
    }
    public void set(double v){
        motor.set(v);
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
