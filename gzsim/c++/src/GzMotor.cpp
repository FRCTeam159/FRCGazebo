#include "GzMotor.h"

std::string GzMotor::topic_base= "~/gazebo/frc/simulator/motor/";
GzMotor::GzMotor(int i,std::shared_ptr<nt::NetworkTable> t) : chnl(i), 
    GzNode(topic_base+std::to_string(i),t)
{
    std::cout<<"GzMotor::GzMotor("<<i<<") gztopic:"<<gz_topic<<std::endl;
}
GzMotor::~GzMotor(){
    std::cout<<"GzMotor::~GzMotor("<<chnl<<")"<<std::endl;
}
void GzMotor::publish(){
    if(stopped())
        return;
    gazebo::msgs::Vector3d msg;
    double x = table->GetNumber("set", 0.0);

    gazebo::msgs::Set(&msg, ignition::math::Vector3d(x, 0, 0));
    pub->Publish(msg);
}
bool GzMotor::connect(){
    bool success=false;

    pub = gz_node->Advertise<gazebo::msgs::Vector3d>(gz_topic);
    success=pub->WaitForConnection(gazebo::common::Time(2, 0));

    if(!success)
        std::cerr << "GzMotor connection failed for channel:"<< chnl << std::endl;
    else
        std::cerr << "GzMotor connection established for channel:"<< chnl << std::endl;
    status=success?CONNECTED:0;

    return success;
}
