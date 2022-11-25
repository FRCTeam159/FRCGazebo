
package frc.robot.objects;

public class TagResult {
    int id;
    int hamming;
    double decision_margin;
    double centerX, centerY;
    double[] corners;

    public TagResult(){
        id=1;
        hamming=1;
        centerX=0;
        centerY=0;
        corners=null;
    }

    public TagResult(
        int id,
        int hamming,
        double decision_margin,
        double centerX,
        double centerY,
        double[] corners)
    {
        this.id = id;
        this.hamming = hamming;
        this.decision_margin = decision_margin;
        this.centerX = centerX;
        this.centerY = centerY;
        this.corners = corners;
    }
    
    public int getId() {
        return id;
    }

    public int getHamming() {
        return hamming;
    }

    public double getDecisionMargin() {
        return decision_margin;
    }
    public double getCenterX() {
        return centerX;
    }
    public double getCenterY() {
        return centerY;
    }
    public double[] getCorners() {
        return corners;
    }
}