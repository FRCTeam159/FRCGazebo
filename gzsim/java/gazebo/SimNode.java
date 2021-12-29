package gazebo;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;

public class SimNode {
    public NetworkTable table;
    //public NetworkTableEntry node;
    public SimNode(){
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        table = inst.getTable("gazebo");
        //node = table.getEntry("GzNode");
    } 
}
