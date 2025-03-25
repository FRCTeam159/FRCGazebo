// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Mat;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSource;
import objects.Camera;

public class Cameras extends Thread {
  CvSource ouputStream;
  public static int kCamWidth = 320;
  public static int kCamHeight = 240;

  Camera cam; // simulatiom camera

  /** Creates a new Camera. */
  public Cameras() {
  }

  public void run() {
    ouputStream = CameraServer.putVideo("Camera", kCamWidth, kCamHeight);
    cam = new Camera(0, kCamWidth, kCamHeight, 80); // specs for Gazebo camera
    cam.start();
    while (!Thread.interrupted()) {
      try {
        Thread.sleep(50);
        Mat mat = cam.getFrame();;
        if (mat != null) {
          ouputStream.putFrame(mat);
        }
      } catch (Exception ex) {
        System.out.println("Cameras exception:" + ex);
      }
    }
  }
}
