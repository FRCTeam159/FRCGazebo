package utils;

import java.util.ArrayList;

public class SimTargetMgr {
    static ArrayList<TagTarget> targets=new ArrayList<>();

    static public  int PERIMETER_TAGS=0;
    static public  int FRONT_TAGS=1;
    static public  int SINGLE_TAG=2;

    static public int type=PERIMETER_TAGS;
    
    static public void init(){
        setSingleTarget();
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

        targets.add(new TagTarget(0,dx,-dy,dz,h45));        // left-front
        targets.add(new TagTarget(1,dx,0,dz,0.0));     // middle-front
        targets.add(new TagTarget(2,dx,dy,dz,-h45));        // right-front
        targets.add(new TagTarget(3,0,dy,dz,h90));      // right-middle
        targets.add(new TagTarget(4,-dx,dy,dz,h45));        // right-back
        targets.add(new TagTarget(5,-dx,0,dz,0));      // middle-back
        targets.add(new TagTarget(6,-dx,-dy,dz,-h45));      // left-back
        targets.add(new TagTarget(7,0,-dy,dz,h90));      // left-middle
    }

    
    static public TagTarget getTarget(int id){
        if(id<0 || id >targets.size())
            return null;
        return targets.get(id);
    }

    
}