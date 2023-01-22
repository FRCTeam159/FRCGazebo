package gazebo;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

public class SimGyro extends SimNode {
    private NetworkTableEntry ctrl_node;
    private NetworkTableEntry yaw_node;
    private NetworkTableEntry pitch_node;
    private NetworkTableEntry roll_node;
    private NetworkTableEntry vel_node;
    private static NetworkTable objects;
    private static NetworkTable objects_chnls;

    private double zero;
    boolean enabled=false;
    boolean resetting=false;
    int chnl;
    public String idstr;
    public SimGyro(int id){
        chnl=id;
        idstr="gyro/"+chnl;
        objects=table.getSubTable("gyro");
        objects_chnls=objects.getSubTable(""+chnl);
        ctrl_node= objects_chnls.getEntry("ctrl");
    
        yaw_node= objects_chnls.getEntry("yaw");     
        pitch_node= objects_chnls.getEntry("pitch");
        roll_node= objects_chnls.getEntry("roll");
        vel_node=objects_chnls.getEntry("velocity");
        yaw_node.setDouble(0.0);
        pitch_node.setDouble(0.0);
        roll_node.setDouble(0.0);
        vel_node.setDouble(0.0);

        ctrl_node.setString("new");
        System.out.println("SimGyro");
    }
    
    public void reset(){
        resetting=true;
        zero=yaw_node.getDouble(0.0);
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
        if(!enabled)
            return 0;
        if(resetting)
          zero=yaw_node.getDouble(0.0);  
        return -(yaw_node.getDouble(0.0)-zero);
    }
    public double getRadians() {
        return -Math.PI*(yaw_node.getDouble(0.0))/180.0;
    }
    public double getYaw() {
        return -yaw_node.getDouble(0.0); 
    }
    public double getPitch() {
        return pitch_node.getDouble(0.0); 
    }
    public double getRoll() {
        return roll_node.getDouble(0.0); 
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
    
}