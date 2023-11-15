// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ShootBall extends CommandBase {

  static final double GATE_DELAY = 1;
  static final double FLYWHEEL_DELAY = 3;
  static final double PUSH_DELAY = 1;
  static final double RESET_DELAY = 0.1;

  double elapsed_time;
  double delta_time;

  boolean timing = false;
  boolean finished = false;
  boolean debug = true;
  Timer timer=new Timer();

  enum State {
    OPEN_GATE,
    TURN_FLYWHEELS_ON,
    PUSH_BALL,
    RESET
  }

  State state;
  State last_state;

  /** Creates a new ShootBall. */
  public ShootBall() {
    timer.start();
    addRequirements(Robot.shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    elapsed_time = 0;
    state = State.OPEN_GATE;
    last_state=state;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (state) {
      case OPEN_GATE:
        openGate();
        break;
      case TURN_FLYWHEELS_ON:
        turnFlywheelsOn();
        break;
      case PUSH_BALL:
        pushBall();
        break;
      case RESET:
        reset();
    }
  }

  void debugPrint(String msg) {
    if (debug)
      System.out.format("ShootBall %1.2f %s\n",elapsed_time,msg);
  }

  void setDeltaTimeout(double t) {
    timing = true;
    elapsed_time += t;
    delta_time = t;
    timer.reset();
  }

  boolean checkTimeout() {
    if (timer.get() > delta_time) {
      timing = false;
      return true;
    }
    return false;
  }

  // *********** State Machine Functions
  // ******************************************************
  // ==========================================================================================
  // ShootBall.OpenGate()
  // ==========================================================================================
  // - openGate - Pinch the Ball
  // ==========================================================================================
  private void openGate() {
    Robot.holder.openGate();
    if (Robot.holder.isGateOpen()) {
      debugPrint("Gate Open ");
      state = State.TURN_FLYWHEELS_ON;
    }
  }

  // ==========================================================================================
  // ShootBall.TurnFlywheelsOn()
  // ==========================================================================================
  // - Turn on the flywheels
  // - Wait for a minimum delay
  // - Wait until flywheels are at target speed
  // - Then Push the ball
  // ==========================================================================================
  private void turnFlywheelsOn() {
    Robot.holder.holdBall();
    if (!timing) {
      debugPrint("Enabling Flywheels ..");
      Robot.shooter.enableFlywheels();
      setDeltaTimeout(FLYWHEEL_DELAY);
    }
    if (Robot.shooter.isAtSpeed()) {
      debugPrint("Flywheels at speed, PushBall started ..");
      state = State.PUSH_BALL;
    } else if (checkTimeout()) {
      debugPrint("Flywheels NOT at speed, shooting anyway ..");
      state = State.PUSH_BALL;
    }
  }

  // ==========================================================================================
  // ShootBall.pushBall()
  // ==========================================================================================
  // - Send a Push request to the Holder
  // - Wait a minimum period to see if the ball has left
  // - If the ball is still present assume the shot failed and the ball is stuck
  // and exit
  // Assume the holder state machine will try to eject the ball
  // - Otherwise, assume the shot succeeded and reset the shooter
  // ==========================================================================================
  private void pushBall() {
    Robot.holder.pushBall();
    if (!timing)
      setDeltaTimeout(PUSH_DELAY);
    else if (checkTimeout()) {
      if (Robot.holder.isBallPresent()) { // Error
        debugPrint("Shoot Error (ball did not leave before timeout) - Exiting");
        finished = true;
      } else
        state = State.RESET;
    }
  }

  //==========================================================================================
// ShootBall::Reset()
//==========================================================================================
// - Reset the shooter and holder
//==========================================================================================
  void reset(){
    debugPrint("Shot Complete - Resetting Subsystems");
    // double min=Robot.shooter.getMinAngle();
    // Robot.shooter.setTargetAngle(min);
    Robot.shooter.reset();
    Robot.holder.reset();
    Robot.loader.reset();
    //Robot.vision.reset();
    finished=true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.shooter.disableFlywheels();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
