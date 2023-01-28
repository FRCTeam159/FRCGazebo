// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.imgcodecs.Imgcodecs;

import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import org.opencv.core.Mat;

/** Add your docs here. */
public class MJpegTest {
    CvSource outputStream;
    Mat mat;
    int cnt=0;
    MjpegServer mjpegServer;
    public static void main(String [] args) {
		new MJpegTest().run();
	}
    MJpegTest(){
        outputStream = new CvSource("Blur", PixelFormat.kMJPEG, 640, 480, 2);
        mjpegServer = new MjpegServer("mjpeg_test", 9000);
        mjpegServer.setSource(outputStream);
        //mat = new mat();
        mat=Imgcodecs.imread("test.jpg");
    }
    public void run(){	
		while (true) {
			try {
                System.out.println("sending image "+cnt++);
                outputStream.putFrame(mat);
				Thread.sleep(500);
			} catch (InterruptedException ex) {
				System.out.println("exception)");
                mjpegServer.close();
				return;
			}			
		}
	}

}
