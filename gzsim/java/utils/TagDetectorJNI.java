// AprilTagDetector.java

package utils;

import java.io.File;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

public class TagDetectorJNI {
    static Mat graymat=new Mat();
    static {
        System.loadLibrary("apriltagjni");
      }
     // Load C++ shared library
     
    private long detector=0; // storage for c++ class pointer
    private int family;
    private static boolean library_loaded=false;

    // native methods 

    private native void detector_create(int fam);
    private native void detector_destroy();
    private native TagResult[] detector_detect(long imgAddr,int rows,int cols);
    private native void detector_test(String filename);

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
         // Creating the empty destination matrix
        graymat=new Mat();
        Imgproc.cvtColor(mat, graymat, Imgproc.COLOR_RGB2GRAY);
        Object[] result = detector_detect(graymat.dataAddr(), graymat.rows(),  graymat.cols());
        return (TagResult[]) result;
    }
    
    public void test(String path){
        File file = new File(path);
        detector_test(file.getAbsolutePath()); 
    }
    public static void main(String[] args) {
        TagDetectorJNI obj=new TagDetectorJNI(0);
        obj.test("hello");
    }  
}