package gazebo;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.interfaces.Gyro;

public class SimGyro extends SimNode implements Gyro {
    private NetworkTableEntry ctrl_node;
    private NetworkTableEntry pos_node;
    private NetworkTableEntry vel_node;
    private static NetworkTable subtable;
    private double zero;
    boolean enabled=false;
    public SimGyro(){
        subtable=table.getSubTable("gyro");
        ctrl_node= subtable.getEntry("ctrl");
        pos_node= subtable.getEntry("heading");
        vel_node= subtable.getEntry("rate");
        ctrl_node.setString("new");
        System.out.println("SimGyro");
    }
    
    public void clear(){
        zero=0;
        ctrl_node.setString("reset");
    }
    public void reset(){
        zero=getHeading();
        ctrl_node.setString("reset");
    }
    public void enable(){
        enabled=true;
        ctrl_node.setString("run");
    }
    public void disable(){
        enabled=false;
        ctrl_node.setString("stop");
    }
    public double getHeading() {
        if(!enabled)
            return 0;
        else
            return pos_node.getDouble(0.0)-zero;
    }
    public double getRate() {
        return vel_node.getDouble(0.0); 
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
        // TODO Auto-generated method stub
        return -getHeading();
    }
}