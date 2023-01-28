// AprilTagDetector.java

package apriltag.jni;

import org.opencv.core.Mat;
import org.opencv.core.Core;

import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

public class TagDetectorJNI {
    static {
        System.loadLibrary("apriltagsim");
    }
  
    private long detector=0; // storage for c++ class pointer
    private int family;
    private static boolean library_loaded=false;

    // native methods 

    private native void detector_create(int fam);
    private native void detector_destroy();
    private native TagResult[] detector_detect(long imgAddr,int rows,int cols,double tw,double fx, double fy, double cx, double cy);
    private native void detector_image_test(long imgAddr,int rows,int cols,boolean pose,boolean timing);

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
        TagResult[] result = detector_detect(graymat.dataAddr(), graymat.rows(),  graymat.cols(),tagwidth,fx,fy,cx,cy);
        return result;
    }
    
    public void test(String path, boolean pose, boolean timing){
        Mat graymat=new Mat();
        Mat mat = Imgcodecs.imread(path);
        Imgproc.cvtColor(mat, graymat, Imgproc.COLOR_RGB2GRAY);
        detector_image_test(graymat.dataAddr(), graymat.rows(),  graymat.cols(),pose,timing);
    }
    public static void main(String[] args) {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        TagDetectorJNI obj=new TagDetectorJNI(0);
        obj.test(System.getenv("GZ_SIM")+"/docs/frame_0000.jpg",true,true);
    }  
}