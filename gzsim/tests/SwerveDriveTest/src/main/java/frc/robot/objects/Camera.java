// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.objects;

import org.opencv.core.Mat;

import edu.wpi.first.cscore.CvSink;
import gazebo.SimCamera;

/** Add your docs here. */
public class Camera implements CameraInterface {

    public int image_width = 640;
    public int image_height = 480;

    // parametes for sim camera and 0.5 m targets
    public double tw=0.4;
    public double hFOV=40;
    public double aspect=((double)image_width)/image_height;
    public double vFOV=hFOV/aspect;
    public double cx=image_width/2.0;
    public double cy=image_height/2.0;
    public double fx=cx/Math.tan(0.5*Math.toRadians(hFOV));
    public double fy=cy/Math.tan(0.5*Math.toRadians(vFOV));
    
    int chnl=0;
    boolean recording=false;
    SimCamera sim_camera;
    MJpegReader video_source;
    CvSink cvSink;
    protected Mat mat=new Mat();
    public Camera(int id){
        setChannel(id);
    }
    public Camera(int id,int w, int h, double f){
        setParams(w,h,f);
        setChannel(id);
    }
    void setChannel(int id){
        chnl=id;
        sim_camera=new SimCamera(id);
        String url=new String("http://localhost:900"+chnl+"/?action=stream");
        video_source=new MJpegReader(url);
        System.out.println("new SimCamera("+chnl+")");
        sim_camera.start();
    }
   
    public void setParams(int w, int h, double f){
        image_width=w;
        image_height=h;
        hFOV=f;
        aspect=((double)image_width)/image_height;
        vFOV=hFOV/aspect;
        cx=image_width/2.0;
        cy=image_height/2.0;
        fx=cx/Math.tan(0.5*Math.toRadians(hFOV));
        fy=cy/Math.tan(0.5*Math.toRadians(vFOV));
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
    public void start() {
        sim_camera.start();
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
