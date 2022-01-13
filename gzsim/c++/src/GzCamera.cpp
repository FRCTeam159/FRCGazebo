#include "GzCamera.h"

std::string GzCamera::topic_base= "~/gazebo/frc/simulator/camera/";
GzCamera::GzCamera(int i, std::shared_ptr<nt::NetworkTable> t) :  chnl(i), 
GzNode(topic_base+std::to_string(i),t) 
{
    std::cout<<"GzCamera::GzCamera("<<i<<") gztopic:"<<gz_topic<<std::endl;
}
GzCamera::~GzCamera(){
    std::cout<<"GzCamera::~GzCamera()"<<std::endl;
}

void GzCamera::callback(const ConstVector3dPtr &msg){
    table->PutNumber("simtime",msg->x());
}

bool GzCamera::connect(){
    bool success=true;
    sub=gz_node->Subscribe(gz_topic,&GzCamera::callback,this);
    pub = gz_node->Advertise<gazebo::msgs::GzString>(gz_topic+"/control");
    success=pub->WaitForConnection(gazebo::common::Time(2, 0));
    if(!success)
        std::cerr << "GzCamera command connection failed"<<std::endl;
    else
        std::cerr << "GzCamera command connection established"<<std::endl;
    status=success?CONNECTED:0;
    return success;
}

void GzCamera::setstate(std::string s){
    std::cout<<"GzCamera::setstate "<<s<<std::endl;
    gazebo::msgs::GzString msg;
    msg.set_data(s);
    pub->Publish(msg);
}
void GzCamera::run(){
    GzNode::run();
    if(status != prev_state)
        setstate("run");
}
void GzCamera::stop(){
    GzNode::stop();
    if(status != prev_state)
        setstate("stop");
}
void GzCamera::reset(){
   GzNode::reset();
   if(status != prev_state)
        setstate("reset");
}
void GzCamera::disable(){
   GzNode::disable();
   if(status != prev_state)
        setstate("disable");
}
void GzCamera::enable(){
   GzNode::enable();
   if(status != prev_state)
        setstate("enable");
}
