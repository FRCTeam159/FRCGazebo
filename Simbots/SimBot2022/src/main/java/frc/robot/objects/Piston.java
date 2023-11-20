// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.objects;

import gazebo.SimPiston;

public class Piston {
    protected int chnl;
    private SimPiston sim_piston;

    public Piston(int id){
        chnl=id;
        sim_piston=new SimPiston(id);
        enable();
    }
    
    public void setForward(){
        sim_piston.forward();
    }
    public void setReverse(){
        sim_piston.reverse();
    }
    public void disable(){
        sim_piston.disable();
    }
    public void enable(){
        sim_piston.enable();
    }
}
