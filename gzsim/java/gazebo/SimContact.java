
package gazebo;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import utils.Averager;

public class SimContact extends SimNode{
    int chnl;
    public String idstr;
    private NetworkTableEntry contact_node;
    private NetworkTableEntry newstate_node;

    private static NetworkTable objects;
    private static NetworkTable channels;
    private NetworkTableEntry ctrl_node;
    boolean enabled=true;
    boolean last_contact;

    Averager averager;
    public SimContact(int id){
        init(id,1);

    }
    public SimContact(int id,int aves){
        init(id,aves);
    }
    void init(int id, int a){
         chnl=id;
        idstr="contact/"+chnl;
        objects=table.getSubTable("contact");
        channels=objects.getSubTable(""+chnl);

        System.out.println("SimContact:"+id);
        contact_node= channels.getEntry("contact");
        newstate_node= channels.getEntry("new-state");
        ctrl_node= channels.getEntry("ctrl");
        ctrl_node.setString("new");
        averager=new Averager(a);
        last_contact=false;
    }

    public void enable(){
        enabled=true;
    }

    public boolean inContact() {
       double val=contact_node.getBoolean(false)?1:0;
       double a=averager.getAve(val);
       return a>=0.5?true:false;
    }
}
