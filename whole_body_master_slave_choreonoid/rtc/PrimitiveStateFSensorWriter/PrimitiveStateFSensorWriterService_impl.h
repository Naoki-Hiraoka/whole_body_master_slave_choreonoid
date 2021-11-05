// -*-C++-*-
#ifndef PrimitiveStateFSensorWriterSERVICESVC_IMPL_H
#define PrimitiveStateFSensorWriterSERVICESVC_IMPL_H

#include "whole_body_master_slave_choreonoid/idl/PrimitiveStateFSensorWriterService.hh"

class PrimitiveStateFSensorWriter;

class PrimitiveStateFSensorWriterService_impl
  : public virtual POA_whole_body_master_slave_choreonoid::PrimitiveStateFSensorWriterService,
    public virtual PortableServer::RefCountServantBase
{
public:
  PrimitiveStateFSensorWriterService_impl();// 実装は.cppファイルの方に書かないと、registerProvider時にSegmentation Faultになる
  ~PrimitiveStateFSensorWriterService_impl();
  void setParams(const whole_body_master_slave_choreonoid::PrimitiveStateFSensorWriterService::PrimitiveStateFSensorWriterParam& i_param);
  void getParams(whole_body_master_slave_choreonoid::PrimitiveStateFSensorWriterService::PrimitiveStateFSensorWriterParam_out i_param);
  //
  void setComp(PrimitiveStateFSensorWriter *i_comp);
private:
  PrimitiveStateFSensorWriter *comp_;
};

#endif
