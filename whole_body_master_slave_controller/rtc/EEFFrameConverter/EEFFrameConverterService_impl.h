// -*-C++-*-
#ifndef EEFFrameConverterSERVICESVC_IMPL_H
#define EEFFrameConverterSERVICESVC_IMPL_H

#include "whole_body_master_slave_controller/idl/EEFFrameConverterService.hh"

class EEFFrameConverter;

class EEFFrameConverterService_impl
  : public virtual POA_whole_body_master_slave_controller::EEFFrameConverterService,
    public virtual PortableServer::RefCountServantBase
{
public:
  EEFFrameConverterService_impl();// 実装は.cppファイルの方に書かないと、registerProvider時にSegmentation Faultになる
  ~EEFFrameConverterService_impl();
  CORBA::Boolean startControl();
  CORBA::Boolean stopControl();
  void setParams(const whole_body_master_slave_controller::EEFFrameConverterService::EEFFrameConverterParam& i_param);
  void getParams(whole_body_master_slave_controller::EEFFrameConverterService::EEFFrameConverterParam_out i_param);
  //
  void setComp(EEFFrameConverter *i_comp);
private:
  EEFFrameConverter *comp_;
};

#endif
