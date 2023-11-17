// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class ExecShooter extends CommandBase implements RobotMap{
  public static final double step_angle=10;
  public static final double incr_angle=0.05;

  
  enum GateState {
		UNDEFINED,
		CLOSED,
		OPEN
	}
	GateState current_gate_state;
	GateState target_gate_state;

  Trigger yButton;
  Trigger aButton;

  Command aimAssist;
  Command shootBall;

  boolean debug = true;


  /** Creates a new ExecShooter. */
  public ExecShooter() {
    addRequirements(Robot.shooter);
    aButton=new JoystickButton(Robot.controller, AIM_ASSIST_BUTTON);
    yButton = new JoystickButton(Robot.controller, SHOOT_BALL_BUTTON);
    aimAssist=new AdjustShot(3.0);
    shootBall=new ShootBall();
  }

  void debugPrint(String msg) {
    if (debug)
      System.out.format("Shooter %s\n",msg);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //System.out.println("ExecShooter.initialize()");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Robot.mode==Robot.Mode.SHOOTING){
      XboxController stick = Robot.controller;
      boolean decStep = stick.getLeftBumperPressed();
      boolean incStep = stick.getRightBumperPressed();
      double left=stick.getLeftTriggerAxis();
      double right=stick.getRightTriggerAxis();
      boolean bButton=stick.getRawButtonPressed(GATE_TOGGLE_BUTTON);
      boolean xButton=stick.getRawButtonPressed(FW_TOGGLE_BUTTON);
      aButton.onTrue(aimAssist);
      yButton.onTrue(shootBall);
      
      if(xButton)
        toggleFW();
      else if(bButton)
        toggleGate();
      else if(decStep) 
        setShooterAngle(-step_angle);
      else if(incStep) 
        setShooterAngle(step_angle);
      else if (left > 0)
        setShooterAngle(-incr_angle*left);
      else if (right > 0)
        setShooterAngle(incr_angle*right);
    }
  }

  private void toggleFW() {
    if(Robot.shooter.flywheelsEnabled())
      Robot.shooter.disableFlywheels();
    else
      Robot.shooter.enableFlywheels();
  }
  private void toggleGate() {
    if (Robot.holder.isGateOpen()){  //If the open gate button is pressed, close the gate.
      current_gate_state=GateState.OPEN;
      target_gate_state=GateState.CLOSED;
      debugPrint("ToggleGate gate is currently Open: Closing ...");
      Robot.holder.closeGate();
    }
    else if (Robot.holder.isGateClosed()){//Otherwise,open.
      current_gate_state=GateState.CLOSED;
      target_gate_state=GateState.OPEN;
      debugPrint("ToggleGate gate is currently Closed: Opening ...");
      Robot.holder.openGate();
    }
    else{
      current_gate_state=GateState.UNDEFINED;
      current_gate_state=GateState.CLOSED;
      Robot.holder.closeGate();
      debugPrint("ToggleGate gate is not at either limit: Closing..");
    }
  }
  private void setShooterAngle(double delta){
    double current=Robot.shooter.getTargetAngle();
	  double max=Robot.shooter.getMaxAngle();
	  double min=Robot.shooter.getMinAngle();

    double target=current+delta;
    target=target>=max?max:target;
    target=target<=min?min:target;

    double push_speed=0.2*target/max;

    Robot.holder.setPushHoldSpeed(push_speed);
	  Robot.shooter.setTargetAngle(target);

  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   // System.out.println("ExecShooter.end("+interrupted+")");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
