// AprilTagDetector.java

package utils;

import java.io.File;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

public class TagDetectorJNI {
    static {
        System.loadLibrary("apriltagjni");
    }
  
    private long detector=0; // storage for c++ class pointer
    private int family;
    private static boolean library_loaded=false;

    // native methods 

    private native void detector_create(int fam);
    private native void detector_destroy();
    private native TagResult[] detector_detect(long imgAddr,int rows,int cols,double tw,double fx, double fy, double cx, double cy);
    private native void detector_image_test(String filename,boolean pose);

    // java methods
  
    public TagDetectorJNI(int fam){
        family=fam;
        create();
    }

    public void create(){
        detector_create(family);
    }
    public void destroy(){
        detector_destroy();
    }
   
    public TagResult[] detect(Mat mat) {   
        return detect(mat,0,0,0,0,0);
    }
    public TagResult[] detect(Mat mat,double tagwidth, double fx, double fy, double cx, double cy) {
        Mat graymat=new Mat();
        Imgproc.cvtColor(mat, graymat, Imgproc.COLOR_RGB2GRAY);
        TagResult[] result = detector_detect(graymat.dataAddr(), graymat.rows(),  graymat.cols(),tagwidth,fx,fy,cy,cy);
        return result;
    }
    
    public void test(String path, boolean pose){
        File file = new File(path);
        detector_image_test(file.getAbsolutePath(),pose); 
    }
    public static void main(String[] args) {
        TagDetectorJNI obj=new TagDetectorJNI(0);
        obj.test(System.getenv("GZ_SIM")+"/docs/apriltag_0_test.jpg",true);
    }  
}