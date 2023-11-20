// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Loader;

// case 1: LOADER mode
// upBtnCmnd2.WhenPressed(new ToggleLoad());
// rightBtnCmnd2.WhenPressed(new StepLoaderAngle(5));
// leftBtnCmnd2.WhenPressed(new StepLoaderAngle(-5));
// downBtnCmnd2.WhenPressed(new ToggleRollers());

public class ExecLoader extends CommandBase implements RobotMap {
  /** Creates a new ExecLoader. */
  public static final int LOAD = 1;
  public static final int LOW = 2;
  public static final int IDLE = 0;

  int state = IDLE;

  public static final double step_angle = 5;
  public static final double incr_angle = 0.05;

  boolean debug = true;

  public ExecLoader() {
    addRequirements(Robot.loader);
  }

  void debugPrint(String msg) {
    if (debug)
      System.out.format("Loader %s\n", msg);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Robot.mode == Robot.Mode.LOADING) {
      XboxController stick = Robot.controller;
      boolean decStep = stick.getLeftBumperPressed();
      boolean incStep = stick.getRightBumperPressed();
      double left = stick.getLeftTriggerAxis();
      double right = stick.getRightTriggerAxis();
      boolean xButton = stick.getRawButtonPressed(ROLLERS_TOGGLE_BUTTON);
      boolean yButton = stick.getRawButtonPressed(LOAD_TOGGLE_BUTTON);
      if (yButton)
        toggleLoad();
      else if (xButton)
        toggleRollers();
      else if (decStep)
        setLoaderAngle(-step_angle);
      else if (incStep)
        setLoaderAngle(step_angle);
      else if (left > 0)
        setLoaderAngle(-incr_angle * left);
      else if (right > 0)
        setLoaderAngle(incr_angle * right);
      // if (!Robot.inAuto()) {
      //   switch (state) {
      //     case LOW:
      //       low();
      //       break;
      //     case LOAD:
      //       load();
      //       break;
      //     case IDLE:
      //       idle();
      //       break;
      //   }
      // }
    }
  }

  private void toggleLoad() {
    if (Robot.loader.loading())
      Robot.loader.setLow();
    else if (!Robot.holder.isBallPresent())
      Robot.loader.loadBall();
  }

  private void setLoaderAngle(double delta) {
    double current = Robot.loader.getTargetAngle();
    double target = current + delta;
    Robot.loader.setLifterAngle(target);
  }

  private void toggleRollers() {
    int initial_state = Robot.loader.getRollorState();
    switch (initial_state) {
      case Loader.ROLLERS_OFF:
        debugPrint("Rollers Off");
        Robot.loader.setRollerState(Loader.ROLLERS_FORWARD);
        break;
      case Loader.ROLLERS_FORWARD:
        debugPrint("Rollers Forward");
        Robot.loader.setRollerState(Loader.ROLLERS_REVERSE);
        break;
      case Loader.ROLLERS_REVERSE:
        debugPrint("Rollers Reverse");
        Robot.loader.setRollerState(Loader.ROLLERS_OFF);
        break;
    }
  }

  void low() {
    if (!Robot.holder.isBallPresent()) {
      state = LOAD;
    } else if (Robot.loader.lifterAtLowerLimit()) {
      state = IDLE;
    } else
      Robot.loader.goToZeroLimitSwitch();
  }

  void load() {
    if (!Robot.loader.loading()) {
      state = LOW;
    } else if (Robot.holder.isBallPresent()) {
      debugPrint("Ball Loaded - Resetting lifter");
      state = LOW;
    } else
      Robot.loader.execLoad();
  }

  void idle() {
    if (Robot.loader.loading() && !Robot.holder.isBallPresent()) {
      state = LOAD;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
