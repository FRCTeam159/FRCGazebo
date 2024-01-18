#include "GzNode.h"

#define BIT_ON(a,b)  a|=(b)
#define BIT_OFF(a,b) a &=~(b)
//#define DEBUG_CNTRL

GzNode::GzNode(std::string s,std::shared_ptr<nt::NetworkTable> t) {
    gz_topic=s;
    table=t;
    status = ENABLED;
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
    else if(s=="enable")
        enable();
    else if(s=="disable")
        disable();
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

void GzNode::disable(){
    clr_status_bit(ENABLED);
#ifdef DEBUG_CNTRL
    if(changed_state)
        std::cout<<name()<<" disable"<<std::endl;
#endif
}
void GzNode::enable(){
    set_status_bit(ENABLED);
#ifdef DEBUG_CNTRL
    if(changed_state)
        std::cout<<name()<<" enable"<<std::endl;
#endif
}
void GzNode::reset(){
    set_status_bit(RESET);
#ifdef DEBUG_CNTRL
    if(changed_state)
        std::cout<<name()<<" reset"<<std::endl;
#endif
 }
void GzNode::stop(){
    clr_status_bit(RUNNING|RESET);
#ifdef DEBUG_CNTRL
    if(changed_state)
        std::cout<<name()<<" stop"<<std::endl;
#endif

}
void GzNode::run(){
    clr_status_bit(RESET);
    set_status_bit(RUNNING);
#ifdef DEBUG_CNTRL
    if(changed_state)
        std::cout<<name()<<" run"<<std::endl;
#endif

}
