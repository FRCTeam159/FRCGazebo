package gazebo;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

public class SimCamera extends SimNode {
    int chnl;
    public String idstr;
    private NetworkTableEntry ctrl_node;
    private static NetworkTable objects;
    private static NetworkTable objects_chnls;
    private boolean recording;

    public SimCamera(int id){
        chnl=id;
        idstr="camera/"+chnl;
        objects=table.getSubTable("camera");
        objects_chnls=objects.getSubTable(""+chnl);
        ctrl_node= objects_chnls.getEntry("ctrl");
        ctrl_node.setString("new");
        recording=false;

        System.out.println("SimCamera:"+id);
    }
    public int getChannel(){
        return chnl;
    }
    public void reset(){
        System.out.println("SimCamera.reset");
        ctrl_node.setString("reset");
        recording=false;
    }
    public void enable(){
        System.out.println("SimCamera.enable");
        ctrl_node.setString("enable");
    }
    public void disable(){
        System.out.println("SimCamera.disable");
        ctrl_node.setString("disable");
        recording=false;
    } 
    public void run(){
        if(!recording){
            System.out.println("SimCamera.record");
            ctrl_node.setString("run");
        }
        recording=true;
    }
    public void start(){
        System.out.println("SimCamera.start");
        ctrl_node.setString("start");  
    }
    public void stop(){
        if(recording){
            System.out.println("SimCamera.stop");
            ctrl_node.setString("stop");
        }
        recording=false;
    }
    public boolean isRecording(){
        return recording;
    }     
}
