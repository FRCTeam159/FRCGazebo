#pragma once

#pragma warning(push, 0)

#include "GzNode.h"

class GzContact : public GzNode{
  static std::string topic_base;
  int chnl;
  void callback(const ConstVector3dPtr &msg);
  public:
    GzContact(int n,std::shared_ptr<nt::NetworkTable> t);
    ~GzContact();
    const std::string name() { return std::string("GzContact"+std::to_string(chnl));}
    bool connect();
};