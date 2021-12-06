// -*-C++-*-
#ifndef CommandAngleFilterSERVICESVC_IMPL_H
#define CommandAngleFilterSERVICESVC_IMPL_H

#include "whole_body_master_slave_choreonoid/idl/CommandAngleFilterService.hh"

class CommandAngleFilter;

class CommandAngleFilterService_impl
  : public virtual POA_whole_body_master_slave_choreonoid::CommandAngleFilterService,
    public virtual PortableServer::RefCountServantBase
{
public:
  CommandAngleFilterService_impl();// 実装は.cppファイルの方に書かないと、registerProvider時にSegmentation Faultになる
  ~CommandAngleFilterService_impl();
  void setParams(const whole_body_master_slave_choreonoid::CommandAngleFilterService::CommandAngleFilterParam& i_param);
  void getParams(whole_body_master_slave_choreonoid::CommandAngleFilterService::CommandAngleFilterParam_out i_param);
  CORBA::Boolean startControl();
  CORBA::Boolean stopControl();
  //
  void setComp(CommandAngleFilter *i_comp);
private:
  CommandAngleFilter *comp_;
};

#endif
