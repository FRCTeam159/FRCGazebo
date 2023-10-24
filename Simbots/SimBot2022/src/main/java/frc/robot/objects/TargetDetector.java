// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.objects;

import org.opencv.core.Mat;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class TargetDetector implements VideoInterface{
    
    protected static final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    protected static final NetworkTable m_target_table=inst.getTable("TargetData");
    protected static final NetworkTable m_target_specs=inst.getTable("TargetSpecs");
   
    protected Mat mat;
    protected static TargetData target=new TargetData();
    protected TargetSpecs target_info=new TargetSpecs();
   
    protected boolean m_annotate=false;
    protected boolean m_connected=false;
    public static boolean show_hsv_threshold=false;

    private NetworkTableEntry ta;
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry tv;
    private NetworkTableEntry tr;
    private NetworkTableEntry tc;

    private NetworkTableEntry idealX;
    private NetworkTableEntry idealY;
    private NetworkTableEntry idealA;
    private NetworkTableEntry useArea;
    private NetworkTableEntry xTol;
    private NetworkTableEntry yTol;
    private NetworkTableEntry aTol;
    private NetworkTableEntry xScale;
    private NetworkTableEntry yScale;
    private NetworkTableEntry aScale;

    public int image_width=640;
    public int image_height=480;

    TargetDetector(){
       setTargetData();
    }
    
    public void outputTargetInImage(){
        m_annotate=true;
    }
    public void outputRawImage(){
        m_annotate=false;
    }
    
    protected void setTargetSpecs(){
        idealX=m_target_specs.getEntry("idealX");
        idealX.setDouble(target_info.idealX);
        idealY=m_target_specs.getEntry("idealY");
        idealY.setDouble(target_info.idealY);
        idealA=m_target_specs.getEntry("idealA");
        idealA.setDouble(target_info.idealA);
        useArea=m_target_specs.getEntry("useArea");
        useArea.setBoolean(target_info.useArea);
        xTol=m_target_specs.getEntry("xTol");
        xTol.setDouble(target_info.xTol);
        yTol=m_target_specs.getEntry("yTol");
        yTol.setDouble(target_info.yTol);
        aTol=m_target_specs.getEntry("aTol");
        aTol.setDouble(target_info.aTol);
        xScale = m_target_specs.getEntry("xScale");
        xScale.setDouble(target_info.xScale);
        yScale = m_target_specs.getEntry("yScale");
        yScale.setDouble(target_info.yScale);
        aScale = m_target_specs.getEntry("aScale");
        aScale.setDouble(target_info.aScale);
    }
    protected void setTargetData(){
        ta= m_target_table.getEntry("ta");
        ta.setDouble(target.ta);
        tx= m_target_table.getEntry("tx");
        tx.setDouble(target.tx);
        ty= m_target_table.getEntry("ty");
        ty.setDouble(target.ty);
        tv= m_target_table.getEntry("tv");
        tv.setBoolean(target.tv); 
        tr= m_target_table.getEntry("tr");
        tr.setDouble(target.tr);     
    }
    
    @Override
    public Mat getFrame(){
        return null;
    }
    
    @Override
    public boolean isConnected() {
        return m_connected;
    }
    @Override
    public void process() {
    }
}
