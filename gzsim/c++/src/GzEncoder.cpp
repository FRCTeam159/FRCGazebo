#include "GzEncoder.h"

std::string GzEncoder::topic_base= "~/gazebo/frc/simulator/encoder/";
GzEncoder::GzEncoder(int i,std::shared_ptr<nt::NetworkTable> t) : chnl(i), 
    GzNode(topic_base+std::to_string(i),t)
{
    std::cout<<"GzEncoder::GzEncoder("<<i<<") gztopic:"<<gz_topic<<std::endl;
    x=y=0;
}
GzEncoder::~GzEncoder(){
    std::cout<<"GzEncoder::~GzEncoder("<<chnl<<")"<<std::endl;
}

void GzEncoder::callback(const ConstVector3dPtr &msg){
    if(running()){
        x=msg->x();
        y=msg->y();
    }
    else{
        x=y=0;
    }
    table->PutNumber("position",x);
    table->PutNumber("velocity",y);
}

bool GzEncoder::connect(){
    bool success=true;
    sub=gz_node->Subscribe(gz_topic,&GzEncoder::callback,this);
    
    pub = gz_node->Advertise<gazebo::msgs::GzString>(gz_topic+"/control");
    success=pub->WaitForConnection(gazebo::common::Time(2, 0));

    if(!success)
        std::cerr << "GzEncoder"<<chnl<<" command connection failed"<< std::endl;
    else
        std::cerr << "GzEncoder"<<chnl<<" command connection established"<< std::endl;
    status=success?CONNECTED:0;
    return success;
}

void GzEncoder::setstate(std::string s){
    //std::cout<<"GzEncoder::setstate "<<s<<std::endl;
    gazebo::msgs::GzString msg;
    msg.set_data(s);
    pub->Publish(msg);
}
void GzEncoder::run(){
    GzNode::run();
    if(status != prev_state)
        setstate("start");
}
void GzEncoder::stop(){
    GzNode::stop();
    if(status != prev_state)
        setstate("stop");
}
void GzEncoder::reset(){  
    GzNode::reset();
    if(status != prev_state)
        setstate("reset");
}