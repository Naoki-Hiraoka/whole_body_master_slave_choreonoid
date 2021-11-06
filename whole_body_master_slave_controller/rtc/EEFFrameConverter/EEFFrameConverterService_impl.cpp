#include "EEFFrameConverterService_impl.h"
#include "EEFFrameConverter.h"

EEFFrameConverterService_impl::EEFFrameConverterService_impl()
{
}

EEFFrameConverterService_impl::~EEFFrameConverterService_impl()
{
}

void EEFFrameConverterService_impl::setComp(EEFFrameConverter *i_comp)
{
  comp_ = i_comp;
}

CORBA::Boolean EEFFrameConverterService_impl::startControl()
{
  return comp_->startControl();
};

CORBA::Boolean EEFFrameConverterService_impl::stopControl()
{
  return comp_->stopControl();
};

void EEFFrameConverterService_impl::setParams(const whole_body_master_slave_controller::EEFFrameConverterService::EEFFrameConverterParam& i_param)
{
  comp_->setParams(i_param);
};

void EEFFrameConverterService_impl::getParams(whole_body_master_slave_controller::EEFFrameConverterService::EEFFrameConverterParam_out i_param)
{
  i_param = whole_body_master_slave_controller::EEFFrameConverterService::EEFFrameConverterParam();
  comp_->getParams(i_param);
};

