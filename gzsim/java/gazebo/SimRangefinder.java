
package gazebo;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
public class SimRangefinder extends SimNode {
    int chnl;
    public String idstr;
    private NetworkTableEntry range_node;

    private static NetworkTable objects;
    private static NetworkTable objects_chnls;
    private NetworkTableEntry ctrl_node;
    boolean enabled=true;

    public SimRangefinder(int id){
        chnl=id;
        idstr="rangefinder/"+chnl;
        objects=table.getSubTable("rangefinder");
        objects_chnls=objects.getSubTable(""+chnl);

        System.out.println("SimRangefinder:"+id);
        ctrl_node= objects_chnls.getEntry("ctrl");
        ctrl_node.setString("new");

        range_node= objects_chnls.getEntry("range");
        range_node.setDouble(0.0);

    }
    public void enable(){
        enabled=true;
        ctrl_node.setString("run");
    }
    public double getDistance() {
        return range_node.getDouble(0.0);
    }
}
