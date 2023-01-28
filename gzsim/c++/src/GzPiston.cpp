#include "GzPiston.h"

std::string GzPiston::topic_base= "~/gazebo/frc/simulator/piston/";
GzPiston::GzPiston(int i,std::shared_ptr<nt::NetworkTable> t) : chnl(i), 
    GzNode(topic_base+std::to_string(i),t)
{
    std::cout<<"GzPiston::GzPiston("<<i<<") gztopic:"<<gz_topic<<std::endl;
}
GzPiston::~GzPiston(){
    std::cout<<"GzPiston::~GzPiston("<<chnl<<")"<<std::endl;
}
void GzPiston::publish(){
    if(stopped())
        return;
    gazebo::msgs::Vector3d msg;
    double x = table->GetNumber("set", 0.0);

    gazebo::msgs::Set(&msg, ignition::math::Vector3d(x, 0, 0));
    pub->Publish(msg);
}
bool GzPiston::connect(){
    bool success=false;

    pub = gz_node->Advertise<gazebo::msgs::Vector3d>(gz_topic);
    success=pub->WaitForConnection(gazebo::common::Time(2, 0));

    if(!success)
        std::cerr << "GzPiston connection failed for channel:"<< chnl << std::endl;
    else
        std::cerr << "GzPiston connection established for channel:"<< chnl << std::endl;
    status=success?CONNECTED:0;

    return success;
}
