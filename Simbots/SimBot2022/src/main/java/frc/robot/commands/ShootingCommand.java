// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Targeting;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooting;
import frc.robot.objects.*;

public class ShootingCommand extends CommandBase {
  public boolean test = false;
  Shooting m_shoot;
  XboxController m_controller;
  Targeting m_aim;
  DriveTrain m_drive;

  public ToggleButton m_toggleShooter;
  public ToggleButton m_toggleIntake;
  public ToggleButton m_switchCamera;

  public JoystickButton aimGo;
  public boolean lastState;
  public int state;
  private final int state_OFF = 0;
  private final int state_LOOKING = 1;
  private final int state_AIM = 2;
  private final int state_FOUND = 3;
  private final int state_SHOOT = 4;
  private final Timer m_timer = new Timer();

  private boolean autoAim;
  private boolean spinBack;
  private boolean manualAim;
  private String last_msg;
  boolean was_front=false;


  public ShootingCommand(Shooting shoot, XboxController controller, Targeting aim, DriveTrain drive) {
    m_drive = drive;
    m_shoot = shoot;
    m_controller = controller;
    m_aim = aim;
    m_toggleShooter = new ToggleButton(new JoystickButton(controller, 3));
    m_toggleIntake = new ToggleButton(new JoystickButton(controller, 2));
    m_switchCamera = new ToggleButton(new JoystickButton(controller, 4));

    aimGo = new JoystickButton(controller, 5);
    addRequirements(shoot, aim);
  }

  void whichCamera() {
    boolean newstate = m_switchCamera.newState();
    if (newstate) {
      boolean is_front = m_aim.frontCamera();
      m_aim.setFrontTarget(!is_front);
      SmartDashboard.putBoolean("Front Camera", !is_front);
    }
  }
  void testIntake() {
    boolean newstate = m_toggleIntake.newState();

    if (newstate) {
      m_timer.reset();
      if (m_shoot.isIntakeOn()) {
        m_shoot.setIntakeHold();
        SmartDashboard.putString("Status", "Intake Holding");
        System.out.println("intake is holding");
      } else {
        m_shoot.setIntakeOn();
        SmartDashboard.putString("Status", "Intake Pushing");
        System.out.println("intake is on");
      }
    }
  }

  void testShooter() {
    boolean newstate = m_toggleShooter.newState();

    if (newstate) {
      m_timer.reset();
      if (m_shoot.isShooterOn()) {
        m_shoot.setShooterOff();
        SmartDashboard.putString("Status", "Shooter is Off");
        System.out.println("shooter is off");
      } else {
        m_shoot.setShooterOn();       
        SmartDashboard.putString("Status", "Shooter is On");
        System.out.println("shooter is on");
      }
    }
   
    //double a=m_shoot.shooterAcceleration();
    //if(Math.abs(a)>0.5)
    //System.out.println(m_shoot.shooterVelocity()+" "+m_shoot.shooterAcceleration());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = state_OFF;
    m_timer.start();
    m_timer.reset();
    autoAim = false;
    spinBack = false;
    manualAim = false;
    SmartDashboard.putString("Status", "Manual Driving");
    m_aim.disable();
    if (test == true)
      m_shoot.setIntakeHold();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (test == true) {
      testIntake();
      testShooter();
      return;
    }
    whichCamera();
    boolean is_front=m_aim.frontCamera();
    if(is_front!=was_front){
      state=state_OFF;
      m_aim.reset();
    }
    was_front=is_front;
    if(is_front)
      front_target_program();
    else
      back_target_program(); 
  }
  // tape target state machine
  private void back_target_program() {
    switch (state) {
      case state_OFF:
        manualAim = false;
        autoAim = false;
        if (m_shoot.isBallCaptured()){
          state = state_FOUND;
          m_shoot.setIntakeHold();
        }
        else 
          m_shoot.setIntakeOff();
        m_shoot.setShooterOff();
        if (m_controller.getRawButtonPressed(2)) {
          showStatus("Intake starting");
          m_shoot.setIntakeOn();
          state = state_LOOKING;
        }
        break;
      case state_LOOKING:
        if (m_controller.getRawButtonPressed(5)) {
          showStatus("Starting Auto intake");
          autoAim = true;
          manualAim = false;
          m_aim.enable();
          state=state_AIM;
        }
        if (m_controller.getRawButtonPressed(6)) {
          showStatus("Starting manual intake");
          autoAim = false;
          manualAim = true;
          state=state_AIM;
        }
        if (m_shoot.isBallCaptured()){
          m_shoot.setIntakeHold();
          state = state_FOUND;
        }
        break;
      case state_AIM:
        if(autoAim){
          m_aim.adjust();
        }
        if (m_shoot.isBallCaptured()) {
          state = state_FOUND;
          m_shoot.setIntakeHold();
          showStatus("Ball captured");
        }
        if (m_controller.getRawButtonPressed(2)) {
          showStatus("Intake cancelled");
          state = state_OFF;
          autoAim = false;
        }
        if (m_controller.getRawButtonPressed(6)) {
          if(autoAim)
          showStatus("Auto Intake cancelled");
          autoAim = false;
        }
        break;
      case state_FOUND:
        if (!m_shoot.isBallCaptured()){
          showStatus("Ball Lost");
          state = state_OFF;
        }
        if (m_controller.getRawButtonPressed(2)){
          showStatus("Intake cancelled");
          state = state_OFF;
        }
      break;
    }
  }
  // tape target state machine
  private void front_target_program() {
    switch (state) {
      case state_OFF:
        manualAim = false;
        autoAim = false;
        m_shoot.setShooterOff();
        if(m_shoot.isBallCaptured())
           m_shoot.setIntakeHold();
         else
           m_shoot.setIntakeOff();
        showStatus("Manual Driving");
        if (m_controller.getRawButtonPressed(2)) {
            m_timer.reset();
            if(!m_shoot.isBallCaptured())
              showStatus("Waiting for ball");
            else{
              showStatus("Ready to aim");
              state = state_AIM;
            }
        }
        break;
     
      case state_AIM:
        //if(!manualAim && m_shoot.isShooterOn())
        //  m_shoot.setShooterOff();
         m_shoot.setIntakeHold();

        if (m_controller.getRawButtonPressed(5)) {
          m_aim.enable();
          showStatus("Starting Auto-Aiming");
          manualAim = false;
          autoAim = true;
        }
        if (!manualAim && m_controller.getRawButtonPressed(6)) {
          showStatus("Starting Manual-aiming");
          autoAim = false;
          manualAim = true;
          m_shoot.setShooterOn();
        }
        if (m_controller.getRawButtonPressed(2)) {
          showStatus("Targeting cancelled");
          state = state_OFF;
          autoAim = false;
        }
        if (autoAim) {
          m_aim.adjust();

          if (m_aim.onTarget()) {
            showStatus("On Target");
            state = state_FOUND;
            autoAim=false;
            m_timer.reset();
            m_aim.disable();
            m_shoot.setShooterOn();
          }
        }
        if (manualAim && m_controller.getRawButtonPressed(1)) {
          showStatus("Shooting");
          m_timer.reset();
          state = state_SHOOT;
        }
        break;
      case state_FOUND:
      if (m_shoot.shooterReady(Shooting.kShootSpeed)){
          state = state_SHOOT;
          showStatus("Shooting");
          m_timer.reset();
        }
        break;
      case state_SHOOT:
        m_shoot.setIntakeOn();
        autoAim = false;
        if (m_timer.get() > 3 || !m_shoot.isBallCaptured()) {
          if (m_shoot.isBallCaptured())
            showStatus("Shot Failed");
          else
            showStatus("Shot Complete");
          System.out.println("Test end");
          state = state_OFF;
          m_timer.reset();
        }
        break;
    }
  }

  private void showStatus(String msg){
    SmartDashboard.putString("Status", msg);
    if(!msg.equals(last_msg))
      System.out.println(msg);
    last_msg=msg;
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
