#include "UltrasonicSubsystem.h"
#include "RobotMap.h"
#include "Commands/UltraUpdate.h"
using namespace frc;

#ifdef SIMULATION
UltrasonicSubsystem::UltrasonicSubsystem() : Subsystem("UltrasonicSubsystem"),
		rangefinder(0){
#else
UltrasonicSubsystem::UltrasonicSubsystem() : Subsystem("UltrasonicSubsystem"),
		port(9600, SerialPort::kOnboard, 8, SerialPort::kParity_None, SerialPort::kStopBits_One),
		dOutput(0){
#endif
	portEnabled=false;

}
void UltrasonicSubsystem::InitDefaultCommand() {
		// Set the default command for a subsystem here.
		SetDefaultCommand(new UltraUpdate());
}

void UltrasonicSubsystem::Disable() {
#ifndef SIMULATION
	dOutput.Set(0);
#endif
	portEnabled=false;
}

void UltrasonicSubsystem::Enable() {
#ifndef SIMULATION
	dOutput.Set(1);
#endif
	portEnabled=true;
}

bool UltrasonicSubsystem::IsEnabled() {
	return portEnabled;
}

double UltrasonicSubsystem::GetDistance() {
	if(!IsEnabled()){
		cout<<"Ultrasonic port not enabled"<<endl;
		return 0.0;
	}
	double value = 0;

#ifdef SIMULATION
	double meters=rangefinder.GetVoltage();
	value=39.3701*meters;
#else
	int ival = 0;
	char buffer [6]={0};
	int inChars = port.Read(buffer,5);
	if(inChars != 5){
		cout<<"unexpected input::"<<buffer<<endl;
		return 0;
	}
	sscanf(buffer, "R%d\n", &ival);
	value=ival;
#endif
	//cout<< "value=" << value << endl;
	frc::SmartDashboard::PutNumber("UltraSonicDistance", value);

	return value;
}

void UltrasonicSubsystem::Init() {
	frc::SmartDashboard::PutNumber("UltraSonicDistance", 0);

}
// Put methods for controlling this subsystem
// here. Call these from Commands.
