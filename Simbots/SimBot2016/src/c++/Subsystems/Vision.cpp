/*
 * Vision.cpp
 *
 *  Created on: Jun 20, 2016
 *      Author: dean
 */

#include <Subsystems/Vision.h>
#include "Commands/UpdateVisionData.h"
#include <math.h>

#define RPD(x) (x)*2*M_PI/360
#define FOV_FACTOR 0.866 //1/(2tan(FOV/2)) = 1/(2*tan(30.0)) = 0.866

#define IMAGE_WIDTH 320
#define IMAGE_HEIGHT 240
#define FOV 60.0

Vision::Vision() : Subsystem("Vision") {
    table = NetworkTable::GetTable("GRIP/targets");
    camera_hoffset=camera_voffset=0;
    SetCameraSpecs(IMAGE_WIDTH,IMAGE_HEIGHT,FOV);
    Log();
}

Vision::~Vision() {
    // TODO Auto-generated destructor stub
}

void Vision::Init() {
    Log();
}

void Vision::ClearTargets(){
    ntargets=0;
    for(int i=0;i<MAX_TARGETS;i++){
        targets[i].area=targets[i].width=targets[i].height=targets[i].centerX=targets[i].centerY=0;
    }
}
void Vision::Update() {
    GetTargets();
}

int Vision::GetTargets() {
    std::vector<double> arr1 = table->GetNumberArray("area",llvm::ArrayRef<double>());
    std::vector<double> arr2 = table->GetNumberArray("centerX",llvm::ArrayRef<double>());
    std::vector<double> arr3 = table->GetNumberArray("centerY",llvm::ArrayRef<double>());
    std::vector<double> arr4 = table->GetNumberArray("width",llvm::ArrayRef<double>());
    std::vector<double> arr5 = table->GetNumberArray("height",llvm::ArrayRef<double>());
    unsigned int size=arr1.size();
    if(arr2.size() != size || arr3.size() != size || arr4.size() != size || arr5.size() != size ){
        std::cout << "Target array error"<< std::endl;
        return ntargets;
    }
    ClearTargets();

    ntargets=size;
    for(int i=0;i<ntargets;i++){
        targets[i].area=arr1[i];
    }
    for(int i=0;i<ntargets;i++){
        targets[i].centerX=arr2[i];
    }
    for(int i=0;i<ntargets;i++){
        targets[i].centerY=arr3[i];
    }
    for(int i=0;i<ntargets;i++){
        targets[i].width=arr4[i];
    }
    for(int i=0;i<ntargets;i++){
        targets[i].height=arr5[i];
    }
    FindClosestTarget();
    Log();
    return ntargets;
}

bool Vision::FindClosestTarget() {
    if(!ntargets)
        return false;
    closest=targets[0];
    for(int i=1;i<ntargets;i++){
        if(targets[i].area>closest.area){
            closest=targets[i];
        }
    }
    return true;
}

void Vision::SetTargetSpecs(double w, double h) {
    target.width=w;
    target.height=h;
}

void Vision::SetCameraSpecs(int w, int h, double f) {
    camera.screen_width=w;
    camera.screen_height=h;
    camera.fov=f;
    camera.fov_factor=1/(2*tan(RPD(f/2.0)));
}

// ===========================================================================================================
// Vision::GetTargetDistance() return straight-line distance to target tick (in inches)
//   - ObjectDistance = ScreenWidth*ObjectWidth/(ObjectScreenWidth*2tan(FOV/2))
//   - where 1/(2tan(FOV/2)) = 1/(2*tan(30.0)) = 0.866
//   - note: for elevated targets height projection of sides will be distorted
// ===========================================================================================================
double Vision::GetTargetDistance() {
    double dw = camera.fov_factor*camera.screen_width*target.width/closest.width;
   // std::cout << "target:"<<dw/12.0<<" offset:"<< calcTargetHOffset(dw)<<std::endl;
    return dw;
}
double Vision::calcTargetHOffset(double range) {
    return camera.fov_factor*camera.screen_width*camera_hoffset/range;
}
double Vision::calcTargetVOffset(double range) {
    return camera.fov_factor*camera.screen_height*camera_voffset/range;
}

double Vision::GetTargetVerticalAngle() {
    double d=GetTargetDistance();
    double adjust=calcTargetVOffset(d);
    double p=0.5*camera.screen_height-closest.centerY+adjust; // v is inverted
    double x=p*camera.fov/camera.screen_height;
    return x;
}

double Vision::GetTargetHorizontalAngle() {
    double d=GetTargetDistance();
    double adjust=calcTargetHOffset(d);
    double p=closest.centerX+adjust-0.5*camera.screen_width;
    double x=p*camera.fov/camera.screen_width;
    return x;
}

void Vision::PrintRawTargetData() {
    std::cout << "area:"  << closest.area
              << " width:" << closest.width
              << " height:"<< closest.height
              << " center:"<<closest.centerX<<","<<closest.centerY
              << std::endl;
}

void Vision::InitDefaultCommand() {
    SetDefaultCommand(new UpdateVisionData());
}


void Vision::SetCameraOffsets(double h, double v) {
    camera_hoffset=h;
    camera_voffset=v;
}

void Vision::Reset() {
	ClearTargets();
    SetAdjusting(false);
    SetAutoTargeting(false);
    SetOnTarget(false);
    Log();
}

void Vision::PrintTargetOffsets() {
}

void Vision::Log() {
    SmartDashboard::PutBoolean("Target Acquired", OnTarget());
    SmartDashboard::PutBoolean("AutoTargeting", GetAutoTargeting());
    SmartDashboard::PutNumber("Targets Available", ntargets);
    SmartDashboard::PutBoolean("Adjusting Shot", Adjusting());
    //PrintRawTargetData();
}

void Vision::AutonomousInit() {
    Reset();
}

void Vision::TeleopInit() {
    Reset();
}

void Vision::DisabledInit() {
    Reset();
}
