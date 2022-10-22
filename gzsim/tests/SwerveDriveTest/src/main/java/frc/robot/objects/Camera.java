// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.objects;

import frc.robot.Robot;
import gazebo.SimCamera;

/** Add your docs here. */
public class Camera implements CameraInterface {
    int chnl=0;
    boolean recording=false;
    SimCamera sim_camera;
    public Camera(int id){
        chnl=id;
        if (!Robot.isReal())
            sim_camera=new SimCamera(id);
    }
    @Override
    public void record() {
        if (!Robot.isReal()){
            sim_camera.run();
            recording=true;
        }
    }
    @Override
    public void stop() {
        if (!Robot.isReal())
            sim_camera.stop();
        recording=false;
    }
    @Override
    public boolean isRecording() {
        return recording;
    }
    @Override
    public int getChannel() {
        return chnl;
    }   
}
