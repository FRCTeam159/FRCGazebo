package gazebo;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

public class SimSwitch extends SimNode{
    int chnl;
    public String idstr;
    private NetworkTableEntry ctrl_node;
    private NetworkTableEntry low_node;
    private NetworkTableEntry high_node;
    private NetworkTableEntry pos_node;


    private static NetworkTable objects;
    private static NetworkTable objects_chnls;
    private double distancePerRotation = 1;
    boolean enabled=false;
    private double zero;

    public SimSwitch(int id){
        chnl=id;
        idstr="switch/"+chnl;
        objects=table.getSubTable("switch");
        objects_chnls=objects.getSubTable(""+chnl);
        ctrl_node= objects_chnls.getEntry("ctrl");
        ctrl_node.setString("new");

        low_node= objects_chnls.getEntry("low");
        low_node.setDouble(0.0);
        high_node= objects_chnls.getEntry("high");
        high_node.setDouble(0.0);
        pos_node= objects_chnls.getEntry("pos");
        pos_node.setDouble(0.0);
        System.out.println("SimSwitch:"+id);
        zero=0;
    }
   
    public void enable(){
        enabled=true;
        ctrl_node.setString("run");
    }
    public void disable(){
        enabled=false;
        ctrl_node.setString("stop");
    }
    public boolean isEnabled(){
        return enabled;
    }
    public double getPosition() {  
        return pos_node.getDouble(0.0);
    }
    double getLow() {  
        return low_node.getDouble(0.0);
    }
    double getHigh() {  
        return high_node.getDouble(0.0);
    }
    public boolean highLimit(){
        if(enabled && getHigh()>0)
            return true;
        return false;
    }
    public boolean lowLimit(){
        if(enabled && getLow()>0)
            return true;
        return false;
    }
}
