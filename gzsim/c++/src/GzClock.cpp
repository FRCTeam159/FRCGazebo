#include "GzClock.h"

std::string GzClock::topic_base= "~/gazebo/frc/simulator/clock";
GzClock::GzClock(std::shared_ptr<nt::NetworkTable> t) :  GzNode(topic_base,t) 
{
    std::cout<<"GzClock::GzClock() gztopic:"<<gz_topic<<std::endl;
}
GzClock::~GzClock(){
    std::cout<<"GzClock::~GzClock()"<<std::endl;
}

void GzClock::callback(const ConstVector3dPtr &msg){
    table->PutNumber("simtime",msg->x());
}

bool GzClock::connect(){
    bool success=true;
    sub=gz_node->Subscribe(gz_topic,&GzClock::callback,this);
    pub = gz_node->Advertise<gazebo::msgs::GzString>(gz_topic+"/control");
    success=pub->WaitForConnection(gazebo::common::Time(2, 0));
    if(!success)
        std::cerr << "GzClock command connection failed"<<std::endl;
    else
        std::cerr << "GzClock command connection established"<<std::endl;
    status=success?CONNECTED:0;
    return success;
}
void GzClock::setstate(std::string s){
    std::cout<<"GzClock::setstate "<<s<<std::endl;
    gazebo::msgs::GzString msg;
    msg.set_data(s);
    pub->Publish(msg);
}
void GzClock::run(){
    GzNode::run();
    if(status != prev_state)
        setstate("start");
}
void GzClock::stop(){
    GzNode::stop();
    if(status != prev_state)
        setstate("stop");
}
void GzClock::reset(){
   GzNode::reset();
   if(status != prev_state)
        setstate("reset");
}
