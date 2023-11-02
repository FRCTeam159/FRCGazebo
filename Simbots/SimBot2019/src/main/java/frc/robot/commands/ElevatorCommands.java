package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Button;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 *
 */
public class ElevatorCommands extends CommandBase implements RobotMap {

  //static public double rate_scale = Elevator.CYCLE_TIME * Elevator.MAX_SPEED; // assumed

  //double setpoint = 0;
  static boolean debug = false;
  Button baseHtButton = new Button(ELEVATOR_RESET_HEIGHT_BUTTON);
  Button incrementButton = new Button(RIGHT_TRIGGER_BUTTON);
  Button decrementButton = new Button(LEFT_TRIGGER_BUTTON);
  Button tiltButton = new Button(ELEVATOR_TILT_BUTTON);

  public ElevatorCommands() {
    addRequirements(Robot.elevator);
  }

  // Called just before this Command runs the first time
  public void initialize() {
    System.out.println("ElevatorCommands.initialize");
    //setpoint = Robot.elevator.getSetpoint();
  }

  // Called repeatedly when this Command is scheduled to run
  public void execute() {
    XboxController stick = Robot.controller;

    double left=stick.getLeftTriggerAxis();
    double right=stick.getRightTriggerAxis();
    // double left = 0.5 * (1 + stick.getRawAxis(LEFT_TRIGGER));
    // double right = 0.5 * (1 + stick.getRawAxis(RIGHT_TRIGGER));
    double stopAxis=stick.getRawAxis(EMERGENCY_STOP);
    double modeAxis=stick.getRawAxis(HATCH_CARGO_MODE);
    
    if(modeAxis < -0.99)
      Robot.hatchMode=false;
    else if (modeAxis >0.99)
      Robot.hatchMode=true;

    Robot.elevator.checkMode();

    if (tiltButton.isPressed()){
      if(Robot.elevator.isTilted())
        Robot.elevator.tiltElevator(true);
      else
        Robot.elevator.tiltElevator(false);
    }
    // if(stopAxis < -0.99)
    //   Robot.elevator.stopElevator();
    // else if (stopAxis > 0.99)
    //   Robot.elevator.enableElevator();
    if (Robot.elevator.isEnabled()){
      if (baseHtButton.isPressed())
        Robot.elevator.resetLevel();
      else if (stick.getRightBumperPressed())
        Robot.elevator.incrLevel();
      else if (stick.getLeftBumperPressed())
        Robot.elevator.decrLevel();
      else if (left > 0)
        Robot.elevator.stepDown(left);
      else if (right > 0)
        Robot.elevator.stepUp(right);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  public boolean isFinished() { // keep going while in teleop
    return false;
  }

  // Called once after isFinished returns true
  public void end(boolean interrupted){
    System.out.println("ElevatorCommands.end");
    //setpoint = 0;
    //Robot.elevator.reset();
    //Robot.elevator.disable();
  }

}
