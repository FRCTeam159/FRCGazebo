package gazebo;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.interfaces.Gyro;

public class SimGyro extends SimNode implements Gyro {
    private NetworkTableEntry ctrl_node;
    private NetworkTableEntry pos_node;
    private NetworkTableEntry vel_node;
    private static NetworkTable subtable;
    public SimGyro(){
        subtable=table.getSubTable("gyro");
        ctrl_node= subtable.getEntry("ctrl");
        pos_node= subtable.getEntry("heading");
        vel_node= subtable.getEntry("rate");
        ctrl_node.setString("new");
        System.out.println("SimGyro");
    }
    public void reset(){
        ctrl_node.setString("reset");
    }
    public void enable(){
        ctrl_node.setString("run");
    }
    public void disable(){
        ctrl_node.setString("stop");
    }
    public double getHeading() {
        return pos_node.getDouble(0.0);
    }
    public double getRate() {
        return vel_node.getDouble(0.0); 
    }
    @Override
    public void close() throws Exception {
        // TODO Auto-generated method stub 
    }
    @Override
    public void calibrate() {
        // TODO Auto-generated method stub 
    }
    @Override
    public double getAngle() {
        // TODO Auto-generated method stub
        return -getHeading();
    }
}