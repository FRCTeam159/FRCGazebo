package gazebo;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

public class SimMotor extends SimNode {
    int chnl;
    public String idstr;
    private NetworkTableEntry set_node;
    private NetworkTableEntry scale_node;
    private NetworkTableEntry ctrl_node;
    private static NetworkTable objects;
    private static NetworkTable channels;
    private double distancePerRotation = 1;
    boolean enabled=false;

    public SimMotor(int id){
        chnl=id;
        idstr="motor/"+chnl;
        objects=table.getSubTable("motor");
        channels=objects.getSubTable(""+chnl);
        ctrl_node= channels.getEntry("ctrl");
        ctrl_node.setString("new");

        set_node= channels.getEntry("set");
        set_node.setDouble(0.0);
        scale_node= channels.getEntry("scale");
        scale_node.setDouble(1.0);
        System.out.println("SimMotor:"+id);

    }
    public void enable(){
        ctrl_node.setString("run");
        enabled=true;
    }
    public void disable(){
        set(0);
        ctrl_node.setString("stop");
        enabled=false;
    }
    public void reset(){
        ctrl_node.setString("reset");
    }
    public void set(double v){
        if(enabled)
            set_node.setDouble(v);
        else 
            set_node.setDouble(0);
    }
    public void setScale(double v){
        scale_node.setDouble(v);
    }
    public void setDistancePerRotation(double d){
        distancePerRotation = d;
    }
    public double getDistancePerRotation() {
        return distancePerRotation;
    }
}
