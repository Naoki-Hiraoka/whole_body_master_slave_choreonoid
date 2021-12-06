#include "CommandAngleFilterService_impl.h"
#include "CommandAngleFilter.h"

CommandAngleFilterService_impl::CommandAngleFilterService_impl()
{
}

CommandAngleFilterService_impl::~CommandAngleFilterService_impl()
{
}

void CommandAngleFilterService_impl::setComp(CommandAngleFilter *i_comp)
{
  comp_ = i_comp;
}

void CommandAngleFilterService_impl::setParams(const whole_body_master_slave_choreonoid::CommandAngleFilterService::CommandAngleFilterParam& i_param)
{
  comp_->setParams(i_param);
};

void CommandAngleFilterService_impl::getParams(whole_body_master_slave_choreonoid::CommandAngleFilterService::CommandAngleFilterParam_out i_param)
{
  i_param = whole_body_master_slave_choreonoid::CommandAngleFilterService::CommandAngleFilterParam();
  comp_->getParams(i_param);
};

CORBA::Boolean CommandAngleFilterService_impl::startControl()
{
  return comp_->startControl();
};

CORBA::Boolean CommandAngleFilterService_impl::stopControl()
{
  return comp_->stopControl();
};
