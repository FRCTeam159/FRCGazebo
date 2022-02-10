
package gazebo;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

public class SimContact extends SimNode{
    int chnl;
    public String idstr;
    private NetworkTableEntry contact_node;
    private NetworkTableEntry newstate_node;

    private static NetworkTable objects;
    private static NetworkTable channels;
    private NetworkTableEntry ctrl_node;
    boolean enabled=true;

    public SimContact(int id){
        chnl=id;
        idstr="contact/"+chnl;
        objects=table.getSubTable("contact");
        channels=objects.getSubTable(""+chnl);

        System.out.println("SimContact:"+id);
        contact_node= channels.getEntry("contact");
        newstate_node= channels.getEntry("new-state");
        ctrl_node= channels.getEntry("ctrl");
        ctrl_node.setString("new");

    }
    
    public void enable(){
        enabled=true;
    }
    public boolean inContact() {
        return contact_node.getBoolean(false);
    }
    public boolean newState() {
        return newstate_node.getBoolean(false);
    }
}
