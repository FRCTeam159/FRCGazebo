// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.objects;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ToggleButton extends SubsystemBase {
  private JoystickButton button;
private boolean resetReady = true;
  private boolean last_state=false;
  /** Creates a new ToggleButton. */
  public ToggleButton(JoystickButton givenbutton){
      button = givenbutton; 
  }
  public boolean newState(){
      boolean state = button.getAsBoolean();
      if(state && resetReady){
          resetReady = false;
          last_state=state;
          return true;
      } else if(!state && !resetReady){
          resetReady = true;
          return false;
      } else{
        return false;
      }
  }
  public boolean getState(){
    return last_state;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
