package gazebo;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

public class SimGyro extends SimNode {
    private NetworkTableEntry ctrl_node;
    private NetworkTableEntry pos_node;
    private NetworkTableEntry vel_node;
    private static NetworkTable subtable;
    private double zero;
    boolean enabled=false;
    boolean resetting=false;
    public SimGyro(){
        subtable=table.getSubTable("gyro");
        ctrl_node= subtable.getEntry("ctrl");
        pos_node= subtable.getEntry("heading");
        vel_node= subtable.getEntry("rate");
        ctrl_node.setString("new");
        System.out.println("SimGyro");
    }
    
    public void reset(){
        resetting=true;
        zero=pos_node.getDouble(0.0);
       //ctrl_node.setString("reset");
    }
    public void enable(){
        enabled=true;
        resetting=false;
        //ctrl_node.setString("run");
    }
    public void disable(){
        enabled=false;
        //ctrl_node.setString("stop");
    }
   
    public double getHeading() {
        if(resetting)
          zero=pos_node.getDouble(0.0);  
        return pos_node.getDouble(0.0)-zero;
    }
    public double getRate() {
        return vel_node.getDouble(0.0); 
    }
    
}