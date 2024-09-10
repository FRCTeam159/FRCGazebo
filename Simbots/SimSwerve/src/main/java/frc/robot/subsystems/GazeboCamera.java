// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Core;
import org.opencv.core.Mat;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import objects.Camera;

public class GazeboCamera extends SubsystemBase {
  static {
    System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
  }
   Camera cam;

  /** Creates a new NoteDetector. 
 * @param kCamHeight 
 * @param kCamWidth */
  public GazeboCamera(int kCamWidth, int kCamHeight) {
    cam = new Camera(0, kCamWidth, kCamHeight, 80); // specs for Gazebo camera
    cam.start();
  }

  public Mat getFrame() {
    return cam.getFrame();
  }
}
