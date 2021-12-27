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
    }
    public void setInverted(){
        sign=-1;
    }
    public void enable(){
        ctrl_node.setString("run");
    }
    public void disable(){
        ctrl_node.setString("stop");
    }
    public void reset(){
        ctrl_node.setString("reset");
    }
    public void setDistancePerRotation(double d){
        distancePerRotation = d;
    }
    public double getDistancePerRotation() {
        return distancePerRotation;
    }
    public double getDistance() {
        return sign*distancePerRotation*pos_node.getDouble(0.0);
    }
    public double getRate() {
        return sign*distancePerRotation*vel_node.getDouble(0.0); 
    }
}
