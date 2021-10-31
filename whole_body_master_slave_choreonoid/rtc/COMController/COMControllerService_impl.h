// -*-C++-*-
#ifndef COMControllerSERVICESVC_IMPL_H
#define COMControllerSERVICESVC_IMPL_H

#include "whole_body_master_slave_choreonoid/idl/COMControllerService.hh"

class COMController;

class COMControllerService_impl
  : public virtual POA_whole_body_master_slave_choreonoid::COMControllerService,
    public virtual PortableServer::RefCountServantBase
{
public:
  COMControllerService_impl();// 実装は.cppファイルの方に書かないと、registerProvider時にSegmentation Faultになる
  ~COMControllerService_impl();
  CORBA::Boolean startControl();
  CORBA::Boolean stopControl();
  void setParams(const whole_body_master_slave_choreonoid::COMControllerService::COMControllerParam& i_param);
  void getParams(whole_body_master_slave_choreonoid::COMControllerService::COMControllerParam_out i_param);
  //
  void setComp(COMController *i_comp);
private:
  COMController *comp_;
};

#endif
