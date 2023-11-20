package frc.robot.commands;

import frc.robot.Robot;

public class DriveForTime extends TimedCommand{
    double speed;
    DriveForTime(double t, double s){
        super(t);
        addRequirements(Robot.driveTrain);
        speed = s;
    }
    @Override
    public void initialize() {
        System.out.println("DriveForTime started");
    }
    @Override
    public void execute() {
        Robot.driveTrain.set(speed,speed);
    }
     // Called once the command ends or is interrupted.
     @Override
     public void end(boolean interrupted) {
         System.out.println("DriveForTime.end");
     }
     // Returns true when the command should end.
     @Override
     public boolean isFinished() {
         return super.isFinished();
     }
}