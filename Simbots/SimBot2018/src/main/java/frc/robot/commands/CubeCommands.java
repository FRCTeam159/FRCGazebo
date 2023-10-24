package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 *
 */
public class CubeCommands extends CommandBase implements Constants, RobotMap {
  boolean armsButtonReleased;
  boolean cubePresent;
  boolean armsOpen = false;

  int state = HOLD;

  public CubeCommands() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    addRequirements(Robot.cubeHandler);
  }

  // Called just before this Command runs the first time
  public void initialize() {
    armsButtonReleased = false;
    cubePresent = Robot.cubeHandler.cubeDetected();
    if (cubePresent)
      Robot.cubeHandler.hold();
    else
      Robot.cubeHandler.grab();
  }

  // Called repeatedly when this Command is scheduled to run
  public void execute() {
    XboxController stick = Robot.controller;
    cubePresent = Robot.cubeHandler.cubeDetected();
    toggleArms();
    if (stick.getRawButton(CUBE_PUSH_BUTTON))
      Robot.cubeHandler.push();
    else if (stick.getRawButton(CUBE_GRAB_BUTTON))
      Robot.cubeHandler.grab();
    else
      Robot.cubeHandler.hold();
    Robot.cubeHandler.spinWheels();
  }

  void toggleArms() {
    XboxController stick = Robot.controller;
    boolean armsButtonPressed = stick.getRawButton(ARM_TOGGLE_BUTTON);
    armsOpen = Robot.cubeHandler.armsOpen();
    if (!armsButtonPressed)
      armsButtonReleased = true;
    else if (armsButtonReleased && armsButtonPressed && armsOpen) {
      Robot.cubeHandler.closeArms();
      armsButtonReleased = false;
    } else if (armsButtonReleased && armsButtonPressed && !armsOpen) {
      Robot.cubeHandler.openArms();
      armsButtonReleased = false;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    System.out.println("DrivePath.end("+interrupted+")");

  }
}
