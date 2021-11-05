#include "SupportEEFFixerService_impl.h"
#include "SupportEEFFixer.h"

SupportEEFFixerService_impl::SupportEEFFixerService_impl()
{
}

SupportEEFFixerService_impl::~SupportEEFFixerService_impl()
{
}

void SupportEEFFixerService_impl::setComp(SupportEEFFixer *i_comp)
{
  comp_ = i_comp;
}

CORBA::Boolean SupportEEFFixerService_impl::startControl()
{
  return comp_->startControl();
};

CORBA::Boolean SupportEEFFixerService_impl::stopControl()
{
  return comp_->stopControl();
};

void SupportEEFFixerService_impl::setParams(const whole_body_master_slave_choreonoid::SupportEEFFixerService::SupportEEFFixerParam& i_param)
{
  comp_->setParams(i_param);
};

void SupportEEFFixerService_impl::getParams(whole_body_master_slave_choreonoid::SupportEEFFixerService::SupportEEFFixerParam_out i_param)
{
  i_param = new whole_body_master_slave_choreonoid::SupportEEFFixerService::SupportEEFFixerParam();
  comp_->getParams(*i_param);
};

void SupportEEFFixerService_impl::applyWrenchDistributionControl(CORBA::Double transitionTime) {
  comp_->applyWrenchDistributionControl(transitionTime);
}
