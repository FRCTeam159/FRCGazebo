package gazebo;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.interfaces.Gyro;

public class SimGyro extends SimNode implements Gyro {
    private NetworkTableEntry ctrl_node;
    private NetworkTableEntry yaw_node;
    private NetworkTableEntry pitch_node;
    private NetworkTableEntry roll_node;
    private NetworkTableEntry vel_node;
    private static NetworkTable objects;
    private static NetworkTable channels;

    public static enum Mode {
		YAW,
		PITCH,
		ROLL,
        THREE_AXIS
	}

    private double zero;
    boolean enabled=false;
    boolean resetting=false;
    int chnl;
    public String idstr;

    public Mode mode=Mode.YAW;
    public SimGyro(int id, Mode m){
       init(id,m);
    }
    public SimGyro(int id){
        init(id,Mode.YAW);
    }

    public void init(int id,Mode m){
        chnl=id;
        mode=m;
        idstr="gyro/"+chnl;
        objects=table.getSubTable("gyro");
        channels=objects.getSubTable(""+chnl);
        ctrl_node= channels.getEntry("ctrl");
    
        yaw_node= channels.getEntry("yaw");     
        pitch_node= channels.getEntry("pitch");
        roll_node= channels.getEntry("roll");
        vel_node=channels.getEntry("velocity");
        yaw_node.setDouble(0.0);
        pitch_node.setDouble(0.0);
        roll_node.setDouble(0.0);
        vel_node.setDouble(0.0);

        ctrl_node.setString("new");
        System.out.println("SimGyro:"+id+" mode:"+mode);

    }
    
    public void reset(double d){
        resetting=true;
        double offset;
        switch(mode){
            default:
            case YAW:
            offset=(yaw_node.getDouble(0.0));
            break;
            case PITCH:
            offset=(pitch_node.getDouble(0.0));
            break;
            case ROLL:
            offset=(roll_node.getDouble(0.0));
            break;
        }
        zero=offset+d;
        System.out.println("Gyro reset offset:"+d+" "+zero);
    }
    public void reset(){
        reset(0);
    }
    public void setEnabled(boolean t){
        enabled=t;
    }
    public boolean isEnabled(){
        return enabled;
    }

    public void enable(){
        ctrl_node.setString("run");
        enabled=true;
        resetting=false;
    }
    public void disable(){
        enabled=false;
    }
   
    public double getHeading() {
        return (yaw_node.getDouble(0.0)-zero);
    }
    public double getRadians() {
        return -Math.PI*(yaw_node.getDouble(0.0))/180.0;
    }
    public double getYaw() {
        return -yaw_node.getDouble(0.0)-zero; 
    }
    public double getPitch() {
        return pitch_node.getDouble(0.0)-zero; 
    }
    public double getRoll() {
        return-roll_node.getDouble(0.0)-zero; 
    }
    public double getY() {
        return pitch_node.getDouble(0.0); 
    }
    public double getX() {
        return roll_node.getDouble(0.0); 
    }
    public double getRate() {
        return vel_node.getDouble(0.0); 
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    @Override
    public void close() throws Exception {
        // TODO Auto-generated method stub
    }

    @Override
    public void calibrate() {
        // TODO Auto-generated method stub
    }

    @Override
    public double getAngle() {
        if(!enabled)
            return 0; 
        switch(mode){
            default:
            case YAW:
                return getHeading();
            case PITCH:
                return getPitch();
            case ROLL:
                return getRoll();
        }
    }
    
}