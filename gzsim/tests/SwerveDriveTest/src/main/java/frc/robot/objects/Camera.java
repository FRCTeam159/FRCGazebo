// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.objects;

import org.opencv.core.Mat;

import edu.wpi.first.cscore.CvSink;
import gazebo.SimCamera;

/** Add your docs here. */
public class Camera implements CameraInterface {
    int chnl=0;
    boolean recording=false;
    SimCamera sim_camera;
    MJpegReader video_source;
    CvSink cvSink;
    protected Mat mat=new Mat();
    public Camera(int id){
        chnl=id;
        sim_camera=new SimCamera(id);
        String url=new String("http://localhost:900"+chnl+"/?action=stream");
        video_source=new MJpegReader(url);
        System.out.println("new SimCamera("+chnl+")");
    }
    @Override
    public void record() {
        sim_camera.run();
        recording=true; 
    }
    @Override
    public void stop() {
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
    @Override
    public Mat getFrame() {
        return video_source.getFrame();
    }
    @Override
    public boolean isConnected() {
        return video_source.isConnected();
    }   
}
