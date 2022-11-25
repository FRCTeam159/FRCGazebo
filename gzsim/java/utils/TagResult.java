
package utils;

import org.opencv.core.Point;

public class TagResult {
    int id;
    int tag_id;
    double margin;
    double centerX, centerY;
    double[][] corners;

    public TagResult(){
        id=0;
        tag_id=0;
        margin=0;
        centerX=0;
        centerY=0;
        corners=null;
    }

    public TagResult(
        int id,
        int tag_id,
        double margin,
        double centerX,
        double centerY,
        double[][] corners)
    {
        this.id = id;
        this.tag_id = tag_id;
        this.margin = margin;
        this.centerX = centerX;
        this.centerY = centerY;
        this.corners = corners;
    }
    
    public TagResult(
        int id,
        int tag_id,
        double margin,
        double centerX,
        double centerY,
        double width,
        double height)
    {
        this.id = id;
        this.tag_id = tag_id;
        this.margin = margin;
        this.centerX = centerX;
        this.centerY = centerY;
        corners=new double[4][2];
        double x1=centerX-0.5*width;
        double x2=centerX+0.5*width;
        double y1=centerY-0.5*height;
        double y2=centerY+0.5*height;
        corners[0][0]=x1;
        corners[0][1]=y1;
        corners[1][0]=x2;
        corners[1][1]=y1;
        corners[2][0]=x2;
        corners[2][1]=y2;
        corners[3][0]=x1;
        corners[3][1]=y2;
    }
    public TagResult(
        int id,
        int tag_id,
        double margin,
        double centerX,
        double centerY,
        double x0,
        double y0,
        double x1,
        double y1,
        double x2,
        double y2,
        double x3,
        double y3
        )
    {
        this.id = id;
        this.tag_id = tag_id;
        this.margin = margin;
        this.centerX = centerX;
        this.centerY = centerY;
        corners=new double[4][2];
        
        corners[0][0]=x0;
        corners[0][1]=y0;
        corners[1][0]=x1;
        corners[1][1]=y1;
        corners[2][0]=x2;
        corners[2][1]=y2;
        corners[3][0]=x3;
        corners[3][1]=y3;
    }
    public int getId() {
        return id;
    }
    public int getTagId() {
        return tag_id;
    }
    public void setCenterPoint(double x, double y){
        centerX=x;
        centerY=y;
    }
    public double getDecisionMargin() {
        return margin;
    }
    public double width() {
        return corners[1][0]-corners[0][0];
    }
    public double height() {
        return corners[0][1]-corners[3][1];
    }
    public double getCenterX() {
        return centerX;
    }
    public Point center(){
        return new Point(centerX,centerY);
    }
    public Point tl(){
        return new Point(corners[0][0],corners[0][1]);
    }
    public Point br(){
        return new Point(corners[2][0],corners[2][1]);
    }
    public Point tr(){
        return new Point(corners[1][0],corners[1][1]);
    }
    public Point bl(){
        return new Point(corners[3][0],corners[3][1]);
    }
    public double getCenterY() {
        return centerY;
    }
    public double[][] getCorners() {
        return corners;
    }
    public String toString(){
        String str=String.format("id:%d tag_id:%d conf:%f cX:%-2.1f cY:%-2.1f w:%d h:%d",
        id,tag_id,margin,centerX,centerY,(int)width(),(int)height()

        );
        return str;

    }
}