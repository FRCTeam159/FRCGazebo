package gazebo;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;

public class SimNode {
    public NetworkTable table;
    protected boolean enabled;
    public SimNode(){
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        //inst.setUpdateRate(0.02);
        table = inst.getTable("gazebo");
        enabled=true;
    } 
    public void enable(){
        enabled=true;
    }
    public void disable(){
        enabled=false;
    }
    public boolean isEnabled(){
        return enabled;
    }
}
