package gazebo;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

public class SimEncoder extends SimNode{
    int chnl;
    public String idstr;
    private NetworkTableEntry ctrl_node;
    private NetworkTableEntry pos_node;
    private NetworkTableEntry vel_node;

    private static NetworkTable objects;
    private static NetworkTable objects_chnls;
    private double distancePerRotation = 1;
    private double sign=1;
    boolean enabled=false;
    private double zero;
    boolean resetting=false;

    public SimEncoder(int id){
        chnl=id;
        idstr="encoder/"+chnl;
        objects=table.getSubTable("encoder");
        objects_chnls=objects.getSubTable(""+chnl);
        ctrl_node= objects_chnls.getEntry("ctrl");
        ctrl_node.setString("new");

        pos_node= objects_chnls.getEntry("position");
        pos_node.setDouble(0.0);
        vel_node= objects_chnls.getEntry("velocity");
        vel_node.setDouble(0.0);
        System.out.println("SimEncoder:"+id);
        zero=0;
    }
    public void setInverted(){
        sign=-1;
    }
    public void enable(){
        enabled=true;
        resetting=false;
        ctrl_node.setString("run");
    }
    public void disable(){
        enabled=false;
        pos_node.setNumber(0);
       // ctrl_node.setString("stop");
    }
   
    public void reset(){
        resetting=true;
        zero=pos_node.getDouble(0.0);
    }
    public void setDistancePerRotation(double d){
        distancePerRotation = d;
    }
    public double getDistancePerRotation() {
        return distancePerRotation;
    }
   
    public double getDistance() {
        if(resetting)
          zero=pos_node.getDouble(0.0);
        return sign*distancePerRotation*(pos_node.getDouble(0.0)-zero);
    }
    public double getRate() {
        if(!enabled)
            return 0;
        else
            return sign*distancePerRotation*vel_node.getDouble(0.0); 
    }
}
