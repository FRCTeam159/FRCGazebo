package gazebo;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

public class SimClock extends SimNode {
    private NetworkTableEntry ctrl_node;
    private NetworkTableEntry time_node;
    private static NetworkTable subtable;
    public SimClock(){
        subtable=table.getSubTable("clock");
        ctrl_node= subtable.getEntry("ctrl");
        ctrl_node.setString("new");
        time_node= subtable.getEntry("simtime");
        System.out.println("SimClock");
    }
    public void reset(){
        ctrl_node.setString("reset");
    }
    public void enable(){
        ctrl_node.setString("run");
    }
    public void disable(){
        ctrl_node.setString("stop");
    }
    public double getTime() {
        return time_node.getDouble(0.0); 
    } 
}
