package frc.robot;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class SubscriberTest{
  public static void main(String[] args) {
   new SubscriberTest().run();
  }

  public void run() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("datatable");
    DoubleSubscriber xSub = table.getDoubleTopic("x").subscribe(0.0);
    DoubleSubscriber ySub = table.getDoubleTopic("y").subscribe(0.0);
    inst.startClient4("example client");
    inst.setServer("localhost");
    //inst.setServerTeam(159);  // where TEAM=190, 294, etc, or use inst.setServer("hostname") or similar
    //inst.startDSClient();  // recommended if running on DS computer; this gets the robot IP from the DS
    while (true) {
      try {
        Thread.sleep(1000);
      } catch (InterruptedException ex) {
        System.out.println("interrupted");
        return;
      }
      double x = xSub.get();
      double y = ySub.get();
      System.out.println("recieved X: " + x + " Y: " + y);
    }
  }

  
}