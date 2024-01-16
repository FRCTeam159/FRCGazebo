package gazebo;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

public class SimClock extends SimNode {
    private NetworkTableEntry ctrl_node;
    private NetworkTableEntry time_node;
    private static NetworkTable subtable;
    private double zero;
    public SimClock(){
        subtable=table.getSubTable("clock");
        ctrl_node= subtable.getEntry("ctrl");
        ctrl_node.setString("new");
        time_node= subtable.getEntry("simtime");
        System.out.println("SimClock");
    }
    public void clear(){
        System.out.println("SimClock.clear");
        ctrl_node.setString("reset");
        zero=0;
    }
    public void reset(){
        zero=time_node.getDouble(0.0);
        System.out.println("SimClock.reset");
    }
    public void enable(){
        System.out.println("SimClock.run");
        ctrl_node.setString("run");
    }
    public void disable(){
        System.out.println("SimClock.stop");
        ctrl_node.setString("stop");
    }
    public boolean started(){
        return time_node.getDouble(0.0)==0?false:true;
    }
    public double getTime() {      
        return time_node.getDouble(0.0)-zero; 
    } 
}
