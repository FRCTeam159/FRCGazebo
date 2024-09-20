package objects;

import gazebo.SimGyro;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

public class Gyro {
     static boolean is_real=false;
     SimGyro sim_gyro=null;
     ADXRS450_Gyro real_gyro=null;

     public Gyro() {
        sim_gyro=new SimGyro(0);
        real_gyro=new ADXRS450_Gyro();
    }
    
    static public void setMode(boolean m){
        is_real=m;
    }

    public void reset(){
        if(is_real)
            real_gyro.reset();
        else
            sim_gyro.reset();

     }
     public double getAngle(){
        if(is_real)
            return -real_gyro.getAngle();
        else
            return sim_gyro.getHeading();
     }   
}
