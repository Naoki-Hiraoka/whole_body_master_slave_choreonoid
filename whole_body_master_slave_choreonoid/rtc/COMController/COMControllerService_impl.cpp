#include "COMControllerService_impl.h"
#include "COMController.h"

COMControllerService_impl::COMControllerService_impl()
{
}

COMControllerService_impl::~COMControllerService_impl()
{
}

void COMControllerService_impl::setComp(COMController *i_comp)
{
  comp_ = i_comp;
}

CORBA::Boolean COMControllerService_impl::startControl()
{
  return comp_->startControl();
};

CORBA::Boolean COMControllerService_impl::stopControl()
{
  return comp_->stopControl();
};

void COMControllerService_impl::setParams(const whole_body_master_slave_choreonoid::COMControllerService::COMControllerParam& i_param)
{
  comp_->setParams(i_param);
};

void COMControllerService_impl::getParams(whole_body_master_slave_choreonoid::COMControllerService::COMControllerParam_out i_param)
{
  i_param = whole_body_master_slave_choreonoid::COMControllerService::COMControllerParam();
  comp_->getParams(i_param);
};

