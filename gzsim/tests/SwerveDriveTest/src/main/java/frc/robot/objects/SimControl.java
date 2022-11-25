
package frc.robot.objects;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class SimControl {
    static NetworkTable table;
    static NetworkTableInstance inst= NetworkTableInstance.getDefault();
    NetworkTableEntry control;
    NetworkTableEntry status;
    public SimControl() { }
    public void init(){
        inst.setUpdateRate(0.02);
        inst.startClient("localhost"); 
        table = inst.getTable("gazebo");
        control = table.getEntry("GzCntrl");
        status = table.getEntry("GzStatus");
        control.setString("init");
    }
    public void run(){
        control.setString("run");
        inst.flush();
    }
    public String getStatus(){
       return status.getString("init");
    }
}
