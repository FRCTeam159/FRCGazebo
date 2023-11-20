// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.objects;

import java.util.ArrayList;

import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/** Add your docs here. */
public class GripDetector extends TargetDetector{
    ArrayList<MatOfPoint> contours;
    ArrayList<Rect> rects = new ArrayList<Rect>();
    MatOfPoint bounding_box;
    Rect best = null;
    Rect biggest = null;
    Rect best_shape = null;
    public double xfov=29.6;
    public double yfov=49.7;
    public double ideal_ratio=2.5;
    public double ave_area;
    public final int BEST_SHAPE=1;
    public final int LARGEST = 0;
    boolean use_bounding_box=false;
    protected boolean use_ave_area=false;

    protected int best_attribute=BEST_SHAPE;

    public void setBestAttribute(int a){
        best_attribute=a;
    }
    public void setIdealRatio(double a){
        ideal_ratio=a;
    }
    public void useBoundingBox(boolean b){
        use_bounding_box=b;
    }
    public void useAveArea(boolean b){
        use_ave_area=b;
    }

    void getTargets(){
        best=null;
        ave_area=0;
        rects.clear();
        double max_area = 0;  
        double best_ratio=10;    
        // find the bounding boxes of all targets
        for (int i = 0; i < contours.size(); i++) {
            MatOfPoint contour = contours.get(i);
            Rect r = Imgproc.boundingRect(contour);
            double area = r.area();
            ave_area+=area;
            rects.add(r);
        }
        if(rects.size()>0)
            ave_area/=rects.size();
        if(use_bounding_box && rects.size()>0){
            int xmin=1000;
            int xmax=0;
            int ymin=1000;
            int ymax=0;
            for (int i = 0; i < rects.size(); i++) {
                Rect r = rects.get(i);
                int x2=r.x+r.width;
                int y2=r.y+r.height;
                xmin=r.x<xmin?r.x:xmin;
                xmax=x2>xmax?x2:xmax;
                ymin=r.y<ymin?r.y:ymin;
                ymax=y2>ymax?y2:ymax;
            }
            Rect b=new Rect((int)xmin,(int)ymin,(int)(xmax-xmin),(int)(ymax-ymin));
            rects.clear();
            rects.add(b);
        }
        for (int i = 0; i < rects.size(); i++) {
            Rect r = rects.get(i);
            double area = r.area();
            
            double ratio_err=Math.abs(r.width/r.height-ideal_ratio);

            if (ratio_err < best_ratio) {
                best_shape = r;
                best_ratio = ratio_err;
            }
            
            if (area > max_area) {
                biggest = r;
                max_area = area;
            }
            if(best_attribute==BEST_SHAPE)
                best=best_shape;
            else
                best=biggest;
        }
        
    }
    void markTargets(){
        for (int i = 0; i < rects.size(); i++) {
            Rect r = rects.get(i);
            Point tl = r.tl();
            Point br = r.br();
            double width = br.x - tl.x;
            double height = br.y - tl.y;
            double xVal = tl.x + 0.5 * width;
            double yVal = tl.y + 0.5 * height;
            double xTweek = 0;//horizontalTweek()/angleFactorWidth;
            double yTweek = 0;//verticalTweek() / angleFactorHeight;
            Point xPoint = new Point(xVal + xTweek, yVal + yTweek);
            if (r == best) {
              Imgproc.rectangle(mat, tl, br, new Scalar(255.0, 255.0, 0.0), 2);
              Imgproc.drawMarker(mat, xPoint, new Scalar(0, 0, 255), Imgproc.MARKER_CROSS, 35, 2, 8);
            } else
              Imgproc.rectangle(mat, tl, br, new Scalar(255.0, 255.0, 255.0), 1);
          }
    }
    protected void setTarget(){
        if(best==null){
            target.tv=false;
            //target.ta=target.tx=target.ty=0;
        }else{
            target.tv=true;
            double image_width=mat.width();
            double image_height=mat.height();
            Point tl = best.tl();
            Point br = best.br();
            double width = br.x - tl.x;
            double height = br.y - tl.y;
            double xcenter=tl.x+0.5*width;
            double ycenter=tl.y+0.5*height;
            target.tx=(xcenter-0.5*image_width)/image_width;
            target.ty=(ycenter-0.5*image_height)/image_height;
            if(use_ave_area)
                target.ta=(ave_area)/(image_width*image_height);
            else
                target.ta=(width*height)/(image_width*image_height);
            target.tr=width/height;
        }
        setTargetData();
    }
}
