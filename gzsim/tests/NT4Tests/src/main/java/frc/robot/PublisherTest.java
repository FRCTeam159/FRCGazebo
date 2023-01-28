// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class PublisherTest {

    public static void main(String[] args) {
        new PublisherTest().run();
    }
    DoublePublisher xPub;
    DoublePublisher yPub;

    double x = 0;
    double y = 0;

    public void run() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        inst.startServer();

        // Get the table within that instance that contains the data. There can
        // be as many tables as you like and exist to make it easier to organize
        // your data. In this case, it's a table called datatable.
        NetworkTable table = inst.getTable("datatable");

        // Start publishing topics within that table that correspond to the X and Y
        // values
        // for some operation in your program.
        // The topic names are actually "/datatable/x" and "/datatable/y".
        xPub = table.getDoubleTopic("x").publish();
        yPub = table.getDoubleTopic("y").publish();
        while (true) {
            try {
              Thread.sleep(1000);
            } catch (InterruptedException ex) {
              System.out.println("interrupted");
              return;
            }
            xPub.set(x);
            yPub.set(y);
            x += 0.05;
            y += 1.0;
            System.out.println("publish X: " + x + " Y: " + y);
          }

    }

}
