package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

public class NTServerTest {
	NetworkTable table;
	NetworkTableEntry test;
	static int count=0;
	NetworkTableInstance inst;

	public static void main(String [] args) {
		new NTServerTest().run();
	}
	public NTServerTest(){
		inst = NetworkTableInstance.getDefault();
		inst.startServer();
		table = inst.getTable("plotdata");
		test = table.getEntry("Server Count");
		System.out.println("server getNetworkMode "+inst.getNetworkMode());
		test.setNumber(count);
	}
	public void run(){	
		while (true) {
			try {
				test.setNumber(count);
				System.out.println("NTServer running "+count);
				Thread.sleep(500);
				count++;
			} catch (InterruptedException ex) {
				System.out.println("exception)");
				return;
			}			
		}
	}
}
