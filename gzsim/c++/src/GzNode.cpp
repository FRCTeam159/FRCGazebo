#include "GzNode.h"

#define BIT_ON(a,b)  a|=(b)
#define BIT_OFF(a,b) a &=~(b)
#define DEBUG_CNTRL

GzNode::GzNode(std::string s,std::shared_ptr<nt::NetworkTable> t) {
    gz_topic=s;
    table=t;
    status = ACTIVE;
    pub=NULL;
    sub=NULL; 
    changed_state=false;
}
void GzNode::getCtrlState(){
    prev_state=status;
    std::string s = table->GetString("ctrl", "");
    
    if(s=="reset")
        reset(); // virtual function
    else if(s=="stop")
        stop();
    else if(s=="run")
        run();
    changed_state=(status != prev_state);
}
void GzNode::set_status_bit(int b){
    BIT_ON(status,b);
}
void GzNode::clr_status_bit(int b){
    BIT_OFF(status,b);
}

const std::string GzNode::name(){
    return "GzNode";
}

void GzNode::reset(){
    set_status_bit(RESET);
#ifdef DEBUG_CNTRL
    if(changed_state)
        std::cout<<name()<<" reset"<<std::endl;
#endif
    //clr_status_bit(RESET);
    //changed_state=false;
}
void GzNode::stop(){
    clr_status_bit(RUNNING|RESET);
#ifdef DEBUG_CNTRL
    if(changed_state)
        std::cout<<name()<<" stop"<<std::endl;
#endif
    //changed_state=false;
}
void GzNode::run(){
    clr_status_bit(RESET);
    set_status_bit(RUNNING);
#ifdef DEBUG_CNTRL
    if(changed_state)
        std::cout<<name()<<" run"<<std::endl;
#endif
    //changed_state=false;
}
