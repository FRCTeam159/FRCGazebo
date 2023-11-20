#include "GzSwitch.h"

std::string GzSwitch::topic_base= "~/gazebo/frc/simulator/switch/";
GzSwitch::GzSwitch(int i,std::shared_ptr<nt::NetworkTable> t) : chnl(i), 
    GzNode(topic_base+std::to_string(i),t)
{
    std::cout<<"GzSwitch::GzSwitch("<<i<<") gztopic:"<<gz_topic<<std::endl;
    low_limit=high_limit=position=0;
}
GzSwitch::~GzSwitch(){
    std::cout<<"GzSwitch::~GzSwitch("<<chnl<<")"<<std::endl;
}

void GzSwitch::callback(const ConstVector3dPtr &msg){
    position=msg->z();
    if(running()){
        low_limit=msg->x();
        high_limit=msg->y();
    }
    else{
        low_limit=high_limit=0;
    }
    table->PutNumber("low",low_limit);
    table->PutNumber("high",high_limit);
    table->PutNumber("pos",position);
}

bool GzSwitch::connect(){
    bool success=true;
    sub=gz_node->Subscribe(gz_topic,&GzSwitch::callback,this);
    
    pub = gz_node->Advertise<gazebo::msgs::GzString>(gz_topic+"/control");
    success=pub->WaitForConnection(gazebo::common::Time(2, 0));

    if(!success)
        std::cerr << "GzSwitch"<<chnl<<" command connection failed"<< std::endl;
    else
        std::cerr << "GzSwitch"<<chnl<<" command connection established"<< std::endl;
    status=success?CONNECTED:0;
    return success;
}

void GzSwitch::setstate(std::string s){
    //std::cout<<"GzSwitch::setstate "<<s<<std::endl;
    gazebo::msgs::GzString msg;
    msg.set_data(s);
    pub->Publish(msg);
}
void GzSwitch::run(){
    GzNode::run();
    if(status != prev_state)
        setstate("start");
}
void GzSwitch::stop(){
    GzNode::stop();
    if(status != prev_state)
        setstate("stop");
}
void GzSwitch::reset(){  
    GzNode::reset();
    if(status != prev_state)
        setstate("reset");
}