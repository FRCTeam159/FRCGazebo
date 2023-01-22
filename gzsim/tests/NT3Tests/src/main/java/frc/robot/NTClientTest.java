package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

public class NTClientTest {
	NetworkTable table;
	NetworkTableEntry test;
	NetworkTableInstance inst;
	static int count=0;
	public boolean server_running=true;
	public static void main(String [] args) {
		NTClientTest client=new NTClientTest();
		client.run();
	}
	public NTClientTest(){
		inst = NetworkTableInstance.getDefault();
	}
	public void run(){
		inst.startClient("localhost");
		table = inst.getTable("plotdata");
		test = table.getEntry("Client Count");
		test.setNumber(count);
		while (true) {
			try {
				test.setNumber(count);
				Thread.sleep(2000);
				count++;
			} catch (InterruptedException ex) {
				System.out.println("exception)");
			}
		}
	}
	
}
