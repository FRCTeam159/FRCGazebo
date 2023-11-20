package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
public class GearPlateToggle extends CommandBase implements RobotMap {
    public GearPlateToggle() {
        addRequirements(Robot.gearSubsystem);
    }
    @Override
    public void execute() {
        boolean gearButtonPressed = Robot.controller.getRawButtonPressed(GEARTOGGLEBUTTON);
        boolean gateOpen = Robot.gearSubsystem.IsOpen();
        if(gearButtonPressed){
            if(gateOpen)
                Robot.gearSubsystem.Close();
            else
                Robot.gearSubsystem.Open();
        }
    }
}
