#include "GzRangefinder.h"

std::string GzRangefinder::topic_base= "~/gazebo/frc/simulator/rangefinder/";
GzRangefinder::GzRangefinder(int i,std::shared_ptr<nt::NetworkTable> t) : chnl(i),
 GzNode(topic_base+std::to_string(i),t) 
{
    std::cout<<"GzRangefinder::GzRangefinder("<<i<<") gztopic:"<<gz_topic<<std::endl;
}
GzRangefinder::~GzRangefinder(){
    std::cout<<"GzRangefinder::~GzRangefinder("<<chnl<<")"<<std::endl;
}

void GzRangefinder::callback(const ConstVector3dPtr &msg){
    table->PutNumber("range",msg->x());      // range 
}

bool GzRangefinder::connect(){
    bool success=true;
    sub=gz_node->Subscribe(gz_topic,&GzRangefinder::callback,this);
    pub = gz_node->Advertise<gazebo::msgs::GzString>(gz_topic+"/control");
    success=pub->WaitForConnection(gazebo::common::Time(2, 0));
    if(!success)
        std::cerr << "GzRangefinder command connection failed"<<std::endl;
    else
        std::cerr << "GzRangefinder command connection established"<<std::endl;
    status=success?CONNECTED:0;
    return success;
}
void GzRangefinder::setstate(std::string s){
    std::cout<<"GzRangefinder::setstate "<<s<<std::endl;
    gazebo::msgs::GzString msg;
    msg.set_data(s);
    pub->Publish(msg);
}
void GzRangefinder::reset(){
   GzNode::reset();
    if(status != prev_state)
        setstate("reset");
}
void GzRangefinder::run(){
    GzNode::run();
    if(status != prev_state)
        setstate("start");
}
void GzRangefinder::stop(){
    GzNode::stop();
    if(status != prev_state)
        setstate("stop");
}
