// -*-C++-*-
#ifndef SupportEEFFixerSERVICESVC_IMPL_H
#define SupportEEFFixerSERVICESVC_IMPL_H

#include "whole_body_master_slave_choreonoid/idl/SupportEEFFixerService.hh"

class SupportEEFFixer;

class SupportEEFFixerService_impl
  : public virtual POA_whole_body_master_slave_choreonoid::SupportEEFFixerService,
    public virtual PortableServer::RefCountServantBase
{
public:
  SupportEEFFixerService_impl();// 実装は.cppファイルの方に書かないと、registerProvider時にSegmentation Faultになる
  ~SupportEEFFixerService_impl();
  CORBA::Boolean startControl();
  CORBA::Boolean stopControl();
  void setParams(const whole_body_master_slave_choreonoid::SupportEEFFixerService::SupportEEFFixerParam& i_param);
  void getParams(whole_body_master_slave_choreonoid::SupportEEFFixerService::SupportEEFFixerParam_out i_param);
  void applyWrenchDistributionControl(CORBA::Double transitionTime);
  void applyTiltControl(CORBA::Double transitionTime);
  //
  void setComp(SupportEEFFixer *i_comp);
private:
  SupportEEFFixer *comp_;
};

#endif
