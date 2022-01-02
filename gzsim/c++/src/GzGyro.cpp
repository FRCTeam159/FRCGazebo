#include "GzGyro.h"

std::string GzGyro::topic_base= "~/gazebo/frc/simulator/gyro";
GzGyro::GzGyro(std::shared_ptr<nt::NetworkTable> t) :  GzNode(topic_base,t) 
{
    std::cout<<"GzGyro::GzGyro() gztopic:"<<gz_topic<<std::endl;
}
GzGyro::~GzGyro(){
    std::cout<<"GzGyro::~GzGyro()"<<std::endl;
}

void GzGyro::callback(const ConstVector3dPtr &msg){
    table->PutNumber("heading",msg->x());
    table->PutNumber("rate",msg->y());
}

bool GzGyro::connect(){
    bool success=true;
    sub=gz_node->Subscribe(gz_topic,&GzGyro::callback,this);
    pub = gz_node->Advertise<gazebo::msgs::GzString>(gz_topic+"/control");
    success=pub->WaitForConnection(gazebo::common::Time(2, 0));
    if(!success)
        std::cerr << "GzGyro command connection failed"<<std::endl;
    else
         std::cerr << "GzGyro command connection established"<<std::endl;
    status=success?CONNECTED:0;
    return success;
}
void GzGyro::setstate(std::string s){
    std::cout<<"GzGyro::setstate "<<s<<std::endl;
    gazebo::msgs::GzString msg;
    msg.set_data(s);
    pub->Publish(msg);
}
void GzGyro::reset(){
   GzNode::reset();
    if(status != prev_state)
        setstate("reset");
}
void GzGyro::run(){
    GzNode::run();
    if(status != prev_state)
        setstate("start");
}
void GzGyro::stop(){
    GzNode::stop();
    if(status != prev_state)
        setstate("stop");
}
