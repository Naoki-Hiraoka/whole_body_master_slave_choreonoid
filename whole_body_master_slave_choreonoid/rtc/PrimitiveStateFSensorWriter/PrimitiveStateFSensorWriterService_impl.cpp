#include "PrimitiveStateFSensorWriterService_impl.h"
#include "PrimitiveStateFSensorWriter.h"

PrimitiveStateFSensorWriterService_impl::PrimitiveStateFSensorWriterService_impl()
{
}

PrimitiveStateFSensorWriterService_impl::~PrimitiveStateFSensorWriterService_impl()
{
}

void PrimitiveStateFSensorWriterService_impl::setComp(PrimitiveStateFSensorWriter *i_comp)
{
  comp_ = i_comp;
}

void PrimitiveStateFSensorWriterService_impl::setParams(const whole_body_master_slave_choreonoid::PrimitiveStateFSensorWriterService::PrimitiveStateFSensorWriterParam& i_param)
{
  comp_->setParams(i_param);
};

void PrimitiveStateFSensorWriterService_impl::getParams(whole_body_master_slave_choreonoid::PrimitiveStateFSensorWriterService::PrimitiveStateFSensorWriterParam_out i_param)
{
  i_param = whole_body_master_slave_choreonoid::PrimitiveStateFSensorWriterService::PrimitiveStateFSensorWriterParam();
  comp_->getParams(i_param);
};
