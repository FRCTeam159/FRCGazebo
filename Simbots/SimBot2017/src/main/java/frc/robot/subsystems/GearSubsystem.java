package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import gazebo.SimPiston;

public class GearSubsystem extends SubsystemBase  {

	SimPiston piston;
	boolean isOpen=false;

	public GearSubsystem(){
		SmartDashboard.putBoolean("GearPlateOpen", isOpen);
		piston = new SimPiston(2);
	}

	public void Open(){
		piston.set(1);
		isOpen=true;
	}
	public void Close(){
		piston.set(-1);
		isOpen=false;
	}
	public boolean IsOpen() { 
		return isOpen;
	}
	@Override
	public void simulationPeriodic(){
		SmartDashboard.putBoolean("GearPlateOpen", isOpen);
	}
}
