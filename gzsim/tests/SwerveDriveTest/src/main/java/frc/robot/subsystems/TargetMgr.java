package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class TargetMgr {
    static ArrayList<TagTarget> targets=new ArrayList<>();

    static final public  int UNKNOWN=0;
    static final public  int SINGLE_TAG=1;
    static final public  int FRONT_TAGS=2;
    static final public  int PERIMETER_TAGS=3;
    static final public  int FIELD_TAGS=4;

    public static double targetSize=0.1524; //0.371; side of inner rectangle of apriltag (meters)

    public static Translation3d field_center_offset=new Translation3d(325.4,157,0); 
    public static Translation3d robot_offset_tag_7=new Translation3d (89,108,15);

    public static Translation3d robot_offset=new Translation3d(0,0,0);
    public static Translation3d field_center=field_center_offset.times(Units.inchesToMeters(1));

    static public int type=UNKNOWN;  // changed by init function
    
    // note: MUST set init function based on "start_gazebo" script

    static boolean field_relative=true;
    static public void init(){
        //setFieldTargets();
        setSingleTarget();
    }
    static public void setFieldRelative(){
        field_relative=true;
    }
    static public void setRobotRelative(){
        field_relative=false;
    }

    static boolean fieldRelative(){
        return field_relative;
    }
    Pose2d getFieldPose(Pose2d robotpose){
        //if(!field_relative)
            return robotpose;     
    }
    static public void setFieldTargets(){
        type=FIELD_TAGS;
        // data taken from inset in field drawing 5 of 7

        Translation3d tags[]={
        new Translation3d(610.77,42.19,18.22), // 1 180
        new Translation3d(610.77,108.19,18.22),// 2 180
        new Translation3d(610.77,174.19,18.22),// 3 180 - drawing chart bug (says y=147)
        new Translation3d(636.96,265.74,27.38),// 4 180
        new Translation3d(14.45,265.74,27.38), // 5 0
        new Translation3d(40.25,174.19,18.22), // 6 0   - drawing chart bug (says y=147)
        new Translation3d(40.25,108.19,18.22), // 7 0
        new Translation3d(40.25,42.19,18.22)   // 8 0
        };

        for(int i=0;i<tags.length;i++)
            tags[i]=tags[i].times(Units.inchesToMeters(1));

        // frc has 0,0 at lower left corner but for Gazebo 0,0 is center of field
        // for display and simulation translate tags to field center
       
        robot_offset=robot_offset_tag_7.times(Units.inchesToMeters(1));

        System.out.println("Robot:"+robot_offset);

        for(int i=0;i<tags.length;i++){
            Translation3d tag=robot_offset.minus(tags[i]);
            double x=tag.getX();
            double y=tag.getY();
            double z=tag.getZ();
            double angle=i>3?90:270;
            angle=Math.toRadians(angle);
            TagTarget tt=new TagTarget(i+1,x,y,z,angle);
            targets.add(tt); 
        }
        System.out.println("tag offsets from FRC origin");
        for(int i=0;i<targets.size();i++){
            Translation3d tag=robotToFRC(targets.get(i).getTranslation());
            System.out.println("id:"+(i+1)+" "+getString(tag));
        }
        System.out.println("\ntag offsets to robot");
        for(int i=0;i<targets.size();i++)
            System.out.println(targets.get(i));

        System.out.println("\noffsets from field center");
        System.out.println("robot "+getString(robot_offset.minus(field_center)));
        for(int i=0;i<targets.size();i++){
            Translation3d tag_frc=robotToFRC(targets.get(i).getTranslation());
            Translation3d tag_center=tag_frc.minus(field_center);    // offsets to gazebo origin
            System.out.println("id:"+(i+1)+" "+getString(tag_center));
        }
    }
    static String getString(Translation3d t){
        return String.format("x:%-2.2f y:%-2.2f z:%-2.2f",t.getX(),t.getY(),t.getZ());
    }
    static Translation3d robotToFRC(Translation3d loc){
        return robot_offset.minus(loc);
    }
    static Translation2d robotToFRC(Translation2d loc){
        return robot_offset.toTranslation2d().minus(loc);
    }
  
    static Translation3d fromField(Translation3d tt){
        //Translation3d atag = tt.times(Units.inchesToMeters(1));
        return tt.minus(field_center);
    }
    static Translation2d fromField(Translation2d tt){
        //Translation3d atag = tt.times(Units.inchesToMeters(1));
        return tt.minus(field_center.toTranslation2d());
    }
    static public void setSingleTarget(){
        type=SINGLE_TAG;
        double dx=8;
        double dz=0.25;
        targets.add(new TagTarget(0,dx,0,dz,0));  // left
    }
    static public void setFrontTargets(){
        type=FRONT_TAGS;
        double dx=8;
        double dy=2;
        double dz=0.25;
        targets.add(new TagTarget(0,dx,-dy,dz,0));  // left
        targets.add(new TagTarget(0,dx,dy,dz,0));   // right
    }
    static public void setPerimTargets(){
        type=PERIMETER_TAGS;
        // set one target on all corners and sides of typical FRC field
        double dx=8.5;
        double dy=4.26;
        double dz=0.25;
    
        double h45=Math.toRadians(45);
        double h90=Math.toRadians(90);

        targets.add(new TagTarget(0,dx,-dy,dz,h45));     // left-front
        targets.add(new TagTarget(1,dx,0,dz,0.0));  // middle-front
        targets.add(new TagTarget(2,dx,dy,dz,-h45));     // right-front
        targets.add(new TagTarget(3,0,dy,dz,h90));     // right-middle
        targets.add(new TagTarget(4,-dx,dy,dz,h45));     // right-back
        targets.add(new TagTarget(5,-dx,0,dz,0));   // middle-back
        targets.add(new TagTarget(6,-dx,-dy,dz,-h45));   // left-back
        targets.add(new TagTarget(7,0,-dy,dz,h90));   // left-middle
    }

    static public int numTargets(){
        switch(type){
            case SINGLE_TAG: return 1;
            case FRONT_TAGS: return 2;
            case PERIMETER_TAGS: return 8;
            case FIELD_TAGS: return 8;
        }
        return type;
    }
    static public TagTarget getTarget(int i){
        int id=(type==FIELD_TAGS)?i-1:i;
        if(id<0 || id >targets.size())
            return null;
        return targets.get(id);
    }

   
    static public class TagTarget {
        Pose3d targetPose;
        int targetID=0;
    
        public TagTarget( int id, Pose3d pose) {
            targetPose = pose;
            targetID = id;
        }
    
        public TagTarget(int id, double x, double y, double z, double a) {
            targetPose = new Pose3d(x,y,z,new Rotation3d(0,0,a));
            targetID = id;
        }
        
        public int getID(){ 
            return targetID;
        }
        public Pose3d getPose(){
            return targetPose;
        }
        public Translation3d getTranslation(){
            return targetPose.getTranslation();
        }
        public double getTargetSize(){
            return targetSize;
        }

        public TagTarget moveTo(Translation3d loc){
            Translation3d trans=loc.minus(targetPose.getTranslation());
            Pose3d pose=new Pose3d(trans,targetPose.getRotation());
            return new TagTarget(targetID,pose);
        }
        public String toString(){
            double angle=targetPose.getRotation().toRotation2d().getDegrees();
            return String.format("id:%d x:%-2.1f y:%2.1f z:%1.2f a:%3.1f",
            targetID,targetPose.getX(),targetPose.getY(),targetPose.getZ(),angle);
        }
       
    }
    
}