#include "Commands/DriveToTarget.h"
#include "RobotMap.h"

#define ADJUST_TIMEOUT 0.5
#define MAX_ANGLE_ERROR 0.5
#define P 0.7
#define I 0.01
#define D 0.0

#define SCALE 0.1
#define MIN_DISTANCE 13

#define DRIVE_TIMEOUT 0.2

DriveToTarget::DriveToTarget() : CommandBase("DriveToTarget"),
    pid(P,I,D,this,this)
{
	Requires(driveTrain.get());
    std::cout << "new DriveToTarget"<< std::endl;
    error=false;
}

// Called just before this Command runs the first time
void DriveToTarget::Initialize() {
	bool istargeting=frc::SmartDashboard::GetBoolean("Targeting", false);
	if(istargeting){
		std::cout << "DriveToTarget: called twice ? "<<std::endl;
		return;
	}

	distance=visionSubsystem->GetTargetDistance();
	frc::SmartDashboard::PutBoolean("Targeting", true);
	SetTimeout(DRIVE_TIMEOUT*distance+1);
    int ntargets = visionSubsystem->GetNumTargets();
    if (ntargets>0){
        std::cout << "DriveToTarget Started ..." << std::endl;
      	pid.Reset();
		pid.SetSetpoint(MIN_DISTANCE);
		pid.Enable();
	    error=false;
	    driveTrain->EnableDrive();
    }
    else{
        error=true;
    	std::cout << "DriveToTarget Error(no targets) canceling ..." << std::endl;
    	End();
    }
}

// Called repeatedly when this Command is scheduled to run
void DriveToTarget::Execute() {
}

// Make this return true when this Command no longer needs to run execute()
bool DriveToTarget::IsFinished() {
	if(error){
		return true;
	}
	if(IsTimedOut()){
		std::cout << "DriveToTarget: Timeout expired"<<std::endl;
        error=true;
		return true;
	}
	//visionSubsystem->GetTargetInfo(target);
#define MAX_DIST_ERR 2
	double d=GetDistance();
	double err=d-MIN_DISTANCE;
	int ntargets=visionSubsystem->GetNumTargets();
	if(ntargets==0 || err<MAX_DIST_ERR)
		return true;
	return false;
}

// Called once after isFinished returns true
void DriveToTarget::End() {
    std::cout << "DriveToTarget End" << std::endl;
	pid.Disable();
	driveTrain->DisableDrive();
	if(!error)
		gearSubsystem->Open();
	frc::SmartDashboard::PutBoolean("Targeting", false);
    error=false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveToTarget::Interrupted() {
    error=true;
    std::cout << "DriveToTarget Interrupted" << std::endl;
	End();
}

double DriveToTarget::PIDGet() {
	return visionSubsystem->GetTargetDistance();
}

//#define DEBUG_COMMAND

double DriveToTarget::GetDistance() {
	double d1=visionSubsystem->GetTargetDistance();
#ifdef USE_ULTRASONIC
	double d2=ultrasonicSubsystem->GetDistance();
	if(d1<18)
		return d2;
#endif
    return d1;
}

void DriveToTarget::PIDWrite(double err) {
	double d=GetDistance();
	int n=visionSubsystem->GetNumTargets();

	if(err!=err){
	    cout<<"DriveToTarget::PIDWrite err=NAN"<<endl;
	    return;
	}

	double df=(d-MIN_DISTANCE)/(distance-MIN_DISTANCE); // fraction of starting distance remaining
	//double afact=(1-df)+0.1; // bias angle correction towards end of travel
	double a=-0.02*df*visionSubsystem->GetTargetAngle();
	// double a=-0.1*df*pow(afact,2.0)*visionSubsystem->GetTargetAngle();
	if(n==0)
		a=0;
    double e=-err;
	double m1=e+a;
	double m2=e-a;
	double mx=m1>m2?m1:m2;
	double scale=mx>1?1/mx:1;
	double l=m2*scale;
	double r=m1*scale;
#ifdef DEBUG_COMMAND
	cout<<"n:"<<n<<" dist:"<<d <<" a:"<<a<<" e:"<<e<<" l:"<<l<<" r:"<<r<<endl;
#endif
	driveTrain->TankDrive(l,r);
}
