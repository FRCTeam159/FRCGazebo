package gazebo;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

public class SimPiston extends SimNode {
    int chnl;
    public String idstr;
    private NetworkTableEntry set_node;
    private NetworkTableEntry ctrl_node;
    private static NetworkTable objects;
    private static NetworkTable channels;

    boolean enabled=false;
    private double sign=1;

    public SimPiston(int id){
        chnl=id;
        idstr="piston/"+chnl;
        objects=table.getSubTable("piston");
        channels=objects.getSubTable(""+chnl);
        ctrl_node= channels.getEntry("ctrl");
        ctrl_node.setString("new");

        set_node= channels.getEntry("set");
        set_node.setDouble(0.0);
        
        System.out.println("SimPiston:"+id);

    }
    public void setInverted(){
        sign=-1;
    }
    public void enable(){
        ctrl_node.setString("run");
        enabled=true;
    }
    public void disable(){
        ctrl_node.setString("stop");
        enabled=false;
    }
    
    public void set(double x){
        if(enabled)
            set_node.setDouble(x);
    }
    public void forward(){
        if(enabled)
            set_node.setDouble(1);
        else 
            set_node.setDouble(0);
    }
    public void reverse(){
        if(enabled)
            set_node.setDouble(-1);
        else 
            set_node.setDouble(0);
    }    
}

