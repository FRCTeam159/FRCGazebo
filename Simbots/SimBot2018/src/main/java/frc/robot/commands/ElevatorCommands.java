package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 *
 */
public class ElevatorCommands extends CommandBase implements RobotMap {

  static public double max_travel = 40; // 40 inches ?
  static public double max_speed = 40; // inches per second
  static public double cycle_time = 0.02; // assumed
  static public double rate_scale = cycle_time * max_speed; // assumed

  double setpoint = 0;
  static boolean debug = false;

  public ElevatorCommands() {
    addRequirements(Robot.elevator);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    System.out.println("Elevator.initialize");
    // Robot.elevator.reset();
    setpoint = 0;
    // Robot.elevator.disable();
    Robot.elevator.enable();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    XboxController stick = Robot.controller;
    boolean goToZeroPressed = stick.getLeftBumperPressed();
    boolean goToSwitchPressed = stick.getRightBumperPressed();

    // double left = 0.5 * (1 + stick.getRawAxis(LEFTTRIGGER));
    // double right = 0.5 * (1 + stick.getRawAxis(RIGHTTRIGGER));
    double left=stick.getLeftTriggerAxis();
    double right=stick.getRightTriggerAxis();
    if(goToSwitchPressed) 
      setpoint=Elevator.SWITCH_HEIGHT;
      //Robot.elevator.setPosition(Elevator.SWITCH_HEIGHT);    
    else if(goToZeroPressed) 
      setpoint=0;
      //Robot.elevator.setPosition(0);
    else if (left > 0)
      decrement(left);
    else if (right > 0)
      increment(right);
    Robot.elevator.setPosition(setpoint);
  }

  void increment(double v) {
    setpoint += v * rate_scale;
  }

  void decrement(double v) {
    setpoint -= v * rate_scale;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() { // keep going while in teleop
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    System.out.println("Elevator.end("+interrupted+")");
    setpoint = 0;
    Robot.elevator.reset();
    Robot.elevator.disable();
  }

}
