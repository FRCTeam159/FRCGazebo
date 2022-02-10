#include "GzContact.h"

std::string GzContact::topic_base= "~/gazebo/frc/simulator/contact/";
GzContact::GzContact(int i,std::shared_ptr<nt::NetworkTable> t) : chnl(i), 
    GzNode(topic_base+std::to_string(i),t)
{
    std::cout<<"GzContact::GzContact("<<i<<") gztopic:"<<gz_topic<<std::endl;
}
GzContact::~GzContact(){
    std::cout<<"GzContact::~GzContact("<<chnl<<")"<<std::endl;
}

void GzContact::callback(const ConstVector3dPtr &msg){
    bool have_contact=msg->x();
    bool new_state=msg->y();
    bool new_contact=msg->z();
    table->PutBoolean("contact",have_contact);
    table->PutBoolean("new-state",new_state);
    table->PutBoolean("new-contact",new_contact);
}

bool GzContact::connect(){
    bool success=true;
    sub=gz_node->Subscribe(gz_topic,&GzContact::callback,this);

    if(!success)
        std::cerr << "GzContact"<<chnl<<" command connection failed"<< std::endl;
    else
        std::cerr << "GzContact"<<chnl<<" command connection established"<< std::endl;
    status=success?CONNECTED:0;
    return success;
}
