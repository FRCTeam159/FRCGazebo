/*
 * AdjustShot.cpp
 *
 *  Created on: Jun 20, 2016
 *      Author: dean
 */

#include <Commands/AdjustShot.h>
#include <Robot.h>
#define ADJUST_TIMEOUT 2.5
#define MAX_ANGLE_ERROR 0.5

#define PI

#ifndef PI
#define VP 0.03
#define VI 0.0005
#define VD 0.05

#define HP 0.05
#define HI 0.001
#define HD 0.2

#define HPIDUPDATERATE 0.01
#define VPIDUPDATERATE 0.04
#else
#define VP 0.03
#define VI 0.0005
#define VD 0.05

#define HP 0.05
#define HI 0.001
#define HD 0.2


#define HPIDUPDATERATE 0.01
#define VPIDUPDATERATE 0.04
#endif
//#define FWSPEED 70
//#define HONLY
//#define VONLY

//#define DEBUG_COMMAND
static bool lastontarget=false;
AdjustShot::AdjustShot() : Command("AdjustShot"),
    Hcntrl(HP,HI,HD),Vcntrl(VP,VI,VD)
{
    Requires(Robot::vision.get());
    std::cout << "new AdjustShot"<< std::endl;
}

void AdjustShot::Initialize() {
     Robot::vision->SetAutoTargeting(true);
     ntargets = Robot::vision->GetTargets();
     if (ntargets>0)
        std::cout << "Auto-targeting Started ..." << std::endl;


    if (ntargets > 0) {
        Hcntrl.Initialize();
        Vcntrl.Initialize();
        //Robot::vision->SetAdjusting(true);
    }
    lastontarget=false;

}
void AdjustShot::Execute() {
#ifdef DEBUG_COMMAND
    std::cout << "Updating targets"<< std::endl;
#endif
    ntargets=Robot::vision->GetTargets();
}

bool AdjustShot::IsFinished() {
    if(ntargets==0){
        Robot::vision->SetOnTarget(false);
        std::cout << "AdjustShot: No targets found"<< std::endl;
        return true;
    }
    if(IsTimedOut()){
        Robot::vision->SetOnTarget(false);
        std::cout << "AdjustShot: Timeout expired"<<std::endl;
        return true;
    }
    bool Vok=Vcntrl.AtTarget();
    bool Hok=Hcntrl.AtTarget();
    bool on_target=Vok && Hok;
    Robot::vision->SetOnTarget(on_target);
    if(!lastontarget&& on_target){
        lastontarget=true;
        return false;
    }
    return(on_target);
}

void AdjustShot::End() {
    Hcntrl.End();
    Vcntrl.End();
    double h=Robot::vision->GetTargetHorizontalAngle();
    double v=Robot::vision->GetTargetVerticalAngle();
    std::cout << TimeSinceInitialized()<< "  End AdjustShot h:"<<h<<" v:"<<v<<std::endl;
    Robot::vision->SetAutoTargeting(false);
    Robot::vision->SetAdjusting(false);
}

// =============== Inner Class AdjustVAngle =============================
// Contains a PID controller to set the changing target for the shooter angle
// - Since shooter->SetTargetAngle is itself controlled by a PID loop
//   this outer adjustment loop needs to run at a slower rate in order to
//   avoid oscillation
// - Disabling the inner shooter PID controller allows for a faster correction cycle
//   but a jump results when the target adjust PID transfers control to the shooter
//   angle PID when this command goes out of scope (PIDController doesn't seem to
//   provide an interface function that can be used to copy the integral correction
//   obtained when one PID is at target to a second controller)
AdjustShot::AdjustVAngle::AdjustVAngle(double P, double I, double D):
   pid(P,I,D,this,this,VPIDUPDATERATE)
{
}

double AdjustShot::AdjustVAngle::PIDGet() {
    return Robot::vision->GetTargetVerticalAngle();
}

void AdjustShot::AdjustVAngle::PIDWrite(double d) {
#ifndef HONLY
    double current=Robot::shooter->GetTargetAngle();
    double target=current-d;
    double max=Robot::shooter->GetMaxAngle();
    double min=Robot::shooter->GetMinAngle();

    target=target>=max?max:target;
    target=target<=min?min:target;
    double push_speed=0.2*target/max;

    Robot::holder->SetPushHoldSpeed(push_speed);
    Robot::shooter->SetTargetAngle(target); // Shooter angle_pid will do the actual correction
#ifdef DEBUG_COMMAND
    double va=Robot::vision->GetTargetVerticalAngle();
    double err=pid.GetError();//Robot::vision->GetTargetVerticalAngle();
    std::cout << "Changing angle:"<<va<<" err:"<<err<<" corr:"<<d<<std::endl;
#endif
#endif
}

bool AdjustShot::AdjustVAngle::AtTarget() {
#ifdef HONLY
    return true;
#else
    double err=Robot::vision->GetTargetVerticalAngle();
    return fabs(err)<=MAX_ANGLE_ERROR;
#endif
}

void AdjustShot::AdjustVAngle::Initialize() {
#ifndef HONLY
    pid.Reset();
    pid.SetAbsoluteTolerance(MAX_ANGLE_ERROR);
    pid.SetSetpoint(0);
    pid.Enable();
#endif
}

void AdjustShot::AdjustVAngle::End() {
    pid.Disable();
}

// =============== Inner Class AdjustHAngle =============================

AdjustShot::AdjustHAngle::AdjustHAngle(double P, double I, double D) :
   pid(P,I,D,this,this,HPIDUPDATERATE)
{
}

double AdjustShot::AdjustHAngle::PIDGet() {
    return Robot::vision->GetTargetHorizontalAngle();
}

void AdjustShot::AdjustHAngle::PIDWrite(double d) {
#ifdef DEBUG_COMMAND
    double ha=Robot::vision->GetTargetHorizontalAngle();
    double err=pid.GetError();//Robot::vision->GetTargetVerticalAngle();
    std::cout << "Changing heading:"<<ha<<" err:"<<err<<" corr:"<<d<<std::endl;
#endif
    Robot::drivetrain->Turn(-d);
}

bool AdjustShot::AdjustHAngle::AtTarget() {
    double err=Robot::vision->GetTargetHorizontalAngle();
    return fabs(err)<=MAX_ANGLE_ERROR;
    //return true;
}

void AdjustShot::AdjustHAngle::Initialize() {
    pid.Reset();
    pid.SetAbsoluteTolerance(MAX_ANGLE_ERROR);
    pid.SetSetpoint(0.0);
    pid.Enable();
}

void AdjustShot::AdjustHAngle::End() {
    pid.Disable();
    Robot::drivetrain->EndTravel();
}
