package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.FuelMonitor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import gazebo.SimMotor;
import gazebo.SimSwitch;

public class FuelSubsystem extends SubsystemBase {
    SimSwitch limits = new SimSwitch(1);
    SimMotor motor=new SimMotor(5);
    public FuelSubsystem(){
        log();
	}
    public void initDefaultCommand() {
		setDefaultCommand(new FuelMonitor());
	}
    public boolean atUpperLimit() {
        return limits.highLimit();
    }
    
    public boolean atLowerLimit() {
        return limits.lowLimit();
    }
    
    public void set(double val){
        motor.set(val);
    }
    public void disable() {
        limits.disable();
        motor.disable();
    }
    
    public void enable() {
        limits.enable();
        motor.enable();
    }
    void log(){
        SmartDashboard.putBoolean("Fuel-highlimit",  atUpperLimit());
        SmartDashboard.putBoolean("Fuel-lowlimit",  atLowerLimit());
    }

    public void simulationPeriodic(){
        log();
    }
}
