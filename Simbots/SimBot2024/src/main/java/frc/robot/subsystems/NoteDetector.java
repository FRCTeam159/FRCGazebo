// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Core;
import org.opencv.core.Mat;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import objects.Camera;

public class NoteDetector extends SubsystemBase {
  static {
    System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
  }
  static Camera cam;

  /** Creates a new NoteDetector. */
  public NoteDetector() {
    cam = new Camera(1, DualCameras.kCamWidth, DualCameras.kCamHeight, 80); // specs for Gazebo camera
    cam.start();
  }

  public static Mat getFrame() {
    return cam.getFrame();
  }
}
