package gazebo;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class SimMotor extends SimNode {
    int chnl;
    public String idstr;
    private NetworkTableEntry pub_node;
    private NetworkTableEntry ctrl_node;
    private static NetworkTable objects;
    private static NetworkTable channels;
    private double distancePerRotation = 1;

    public SimMotor(int id){
        chnl=id;
        idstr="motor/"+chnl;
        objects=table.getSubTable("motor");
        channels=objects.getSubTable(""+chnl);
        ctrl_node= channels.getEntry("ctrl");
        ctrl_node.setString("new");

        pub_node= channels.getEntry("pub");
        pub_node.setDouble(0.0);
        System.out.println("SimMotor:"+id);
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
    public void set(double v){
        pub_node.setDouble(v);
    }
    public void setDistancePerRotation(double d){
        distancePerRotation = d;
    }
    public double getDistancePerRotation() {
        return distancePerRotation;
    }
}
