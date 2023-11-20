package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotMap;

/**
 *
 */
public class DriveWithGamepad extends CommandBase implements RobotMap {
  public static double MINTHRESHOLD = 0.3;
  public static double MINOUTPUT = 0;
  boolean gearButtonReleased;
  public static double turnScale = 0.8;
  private boolean started=false;

  public static final int LEFT_JOYSTICK = 1;
  public static final int RIGHT_JOYSTICK = 4;

  public DriveWithGamepad() {
    // Use requires() here to declare subsystem dependencies
    addRequirements(Robot.driveTrain);
   // SmartDashboard.putNumber("Turn Scale", turnScale);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    System.out.println("DriveWithGamepad::initialize()");
    Robot.driveTrain.enable();
    Robot.driveTrain.arcadeDrive(0, 0, SQUARE_INPUTS);
    // Robot.driveTrain.setHighGear();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {

    toggleGear();
    double zs = Robot.controller.getRawAxis(LEFT_JOYSTICK);
    double xs = Robot.controller.getRawAxis(RIGHT_JOYSTICK);
 
    double zoff=0.35;

    if (Math.abs(zs) < zoff) 
      zs = 0;
    else if(zs>0)
      zs=(zs-zoff)/(1-zoff);

    if (Math.abs(xs) < 0.1) 
      xs = 0;
    if(!started && (Math.abs(zs)>0 || Math.abs(xs)>0)){
      Robot.driveTrain.enable();
      started=true;
    }
    //Robot.driveTrain.arcadeDrive(0, 0, SQUARE_INPUTS);

    Robot.driveTrain.arcadeDrive(-zs, -xs, SQUARE_INPUTS);

    // Get axis values
    /* 
    Joystick stick = OI.stick;
    double move = 0; // left stick - drive
    double rotate = 0; // right stick - rotate
    move = -stick.getRawAxis(1); // left stick - drive
    rotate = -stick.getRawAxis(4); // right stick - rotate
    turnScale = SmartDashboard.getNumber("Turn Scale", turnScale);
    rotate *= Math.abs(move) * (1 - turnScale) + turnScale;
   
    Robot.driveTrain.arcadeDrive(move, rotate, SQUARE_INPUTS);
    */
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    System.out.println("DriveWithGamepad::end("+interrupted+")");
  }

  void toggleGear() {
    XboxController stick = Robot.controller;
   // boolean gearButtonPressed = stick.getRawButton(RobotMap.GEAR_TOGGLE_BUTTON);
    boolean gearButtonPressed = stick.getYButtonPressed();

    boolean inLow = Robot.driveTrain.inLowGear();
    if (!gearButtonPressed)
      gearButtonReleased = true;
    else if (gearButtonReleased && gearButtonPressed && inLow) {
      Robot.driveTrain.setHighGear();
      gearButtonReleased = false;
    } else if (gearButtonReleased && gearButtonPressed && !inLow) {
      Robot.driveTrain.setLowGear();
      gearButtonReleased = false;
    }
  }

  /**
   * @param minThreshold
   * @param minOutput
   * @param input
   * @return
   */
  protected double quadDeadband(double minThreshold, double minOutput, double input) {
    if (input > minThreshold) {
      return ((((1 - minOutput) / ((1 - minThreshold) * (1 - minThreshold)))
          * ((input - minThreshold) * (input - minThreshold))) + minOutput);
    } else {
      if (input < (-1 * minThreshold)) {
        return (((minOutput - 1) / ((minThreshold - 1) * (minThreshold - 1)))
            * ((minThreshold + input) * (minThreshold + input))) - minOutput;
      } else {
        return 0;
      }
    }
  }
}
