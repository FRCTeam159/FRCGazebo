// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import gazebo.SimMotor;

/**
 * This sample program shows how to control a motor using a joystick. In the operator control part
 * of the program, the joystick is read and the value is written to the motor.
 *
 * <p>Joystick analog values range from -1 to 1 and speed controller inputs also range from -1 to 1
 * making it easy to work together.
 */
public class Robot extends TimedRobot {

  public static NetworkTable table;
  
  public static final int LEFT_JOYSTICK = 1;
  public static final int RIGHT_JOYSTICK = 4;

  private SimMotor gzmotor1;
  private SimMotor gzmotor2;
  private SimControl gzcontrol;

  private final XboxController m_Controller = new XboxController(0);

  @Override
  public void robotInit() {
    System.out.println("robotInit()");
    gzcontrol=new SimControl();
    gzmotor1=new SimMotor(1);
    gzmotor2=new SimMotor(2);
  }

  @Override
  public void teleopPeriodic() {}
  @Override
  public void simulationInit() {
    System.out.println("simulationInit()");
    gzcontrol.run();
  }
  public void disabledInit() {
    //gzcontrol.setState("stop");
  }
  @Override
  public void simulationPeriodic() {
    double zs = -m_Controller.getRawAxis(LEFT_JOYSTICK);
    double xs = m_Controller.getRawAxis(RIGHT_JOYSTICK);
    if (Math.abs(zs) < 0.1)
      zs = 0;
    if (Math.abs(xs) < 0.1) 
      xs = 0;
    arcadeDrive(zs,xs);
  }
  public void arcadeDrive(double moveValue, double turnValue) {
    
		double leftMotorOutput;
		double rightMotorOutput;
		
		if (moveValue > 0.0) {
			if (turnValue > 0.0) {
				leftMotorOutput = Math.max(moveValue, turnValue);
				rightMotorOutput = moveValue - turnValue;
			} else {
				leftMotorOutput = moveValue + turnValue;
				rightMotorOutput = Math.max(moveValue, -turnValue);
			}
		} else {
			if (turnValue > 0.0) {
				leftMotorOutput = moveValue + turnValue;
				rightMotorOutput = -Math.max(-moveValue, turnValue);
			} else {
				leftMotorOutput = -Math.max(-moveValue, -turnValue);
				rightMotorOutput = moveValue - turnValue;
			}
		}
    gzmotor1.set(leftMotorOutput);
    gzmotor2.set(rightMotorOutput);
	}
}
