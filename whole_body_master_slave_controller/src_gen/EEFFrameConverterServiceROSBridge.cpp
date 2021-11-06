// -*- C++ -*-
/*!
 * @file  EEFFrameConverterServiceROSBridge.cpp * @brief  * $Date$ 
 *
 * $Id$ 
 */
#include "EEFFrameConverterServiceROSBridge.h"

// Module specification
// <rtc-template block="module_spec">
static const char* eefframeconverterservicerosbridge_spec[] =
  {
    "implementation_id", "EEFFrameConverterServiceROSBridge",
    "type_name",         "EEFFrameConverterServiceROSBridge",
    "description",       "",
    "version",           "1.0.0",
    "vendor",            "",
    "category",          "",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    ""
  };
// </rtc-template>

EEFFrameConverterServiceROSBridge::EEFFrameConverterServiceROSBridge(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_EEFFrameConverterServicePort("EEFFrameConverterService")

    // </rtc-template>
{
}

EEFFrameConverterServiceROSBridge::~EEFFrameConverterServiceROSBridge()
{
}


RTC::ReturnCode_t EEFFrameConverterServiceROSBridge::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers

  // Set OutPort buffer

  // Set service provider to Ports

  // Set service consumers to Ports
  nh = ros::NodeHandle("~");
  std::string port_name = "service0";
  nh.getParam("service_port", port_name);
  m_EEFFrameConverterServicePort.registerConsumer(port_name.c_str(), "EEFFrameConverterService", m_service0);

  // Set CORBA Service Ports
  addPort(m_EEFFrameConverterServicePort);

  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable

  // </rtc-template>
  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t EEFFrameConverterServiceROSBridge::onFinalize()
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t EEFFrameConverterServiceROSBridge::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t EEFFrameConverterServiceROSBridge::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t EEFFrameConverterServiceROSBridge::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t EEFFrameConverterServiceROSBridge::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t EEFFrameConverterServiceROSBridge::onExecute(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t EEFFrameConverterServiceROSBridge::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t EEFFrameConverterServiceROSBridge::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t EEFFrameConverterServiceROSBridge::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t EEFFrameConverterServiceROSBridge::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t EEFFrameConverterServiceROSBridge::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


extern "C"
{
 
  void EEFFrameConverterServiceROSBridgeInit(RTC::Manager* manager)
  {
    coil::Properties profile(eefframeconverterservicerosbridge_spec);
    manager->registerFactory(profile,
                             RTC::Create<EEFFrameConverterServiceROSBridge>,
                             RTC::Delete<EEFFrameConverterServiceROSBridge>);
  }
  
};




RTC::ReturnCode_t EEFFrameConverterServiceROSBridge::onExecute(RTC::UniqueId ec_id) {
  ros::spinOnce();
  return RTC::RTC_OK;
}

RTC::ReturnCode_t EEFFrameConverterServiceROSBridge::onActivated(RTC::UniqueId ec_id) {
  _srv0 = nh.advertiseService("setParams", &EEFFrameConverterServiceROSBridge::setParams, this);
  _srv1 = nh.advertiseService("getParams", &EEFFrameConverterServiceROSBridge::getParams, this);
  _srv2 = nh.advertiseService("startControl", &EEFFrameConverterServiceROSBridge::startControl, this);
  _srv3 = nh.advertiseService("stopControl", &EEFFrameConverterServiceROSBridge::stopControl, this);
  return RTC::RTC_OK;
}

RTC::ReturnCode_t EEFFrameConverterServiceROSBridge::onDeactivated(RTC::UniqueId ec_id) {
  _srv0.shutdown();
  _srv1.shutdown();
  _srv2.shutdown();
  _srv3.shutdown();
  return RTC::RTC_OK;
}



template<typename T,typename S>
void genseq(S& s, int size, _CORBA_Unbounded_Sequence<T>* p){
  s = S(size,size,S::allocbuf(size), 1);}
template<typename T,typename S,int n>
void genseq(S& s, int size, _CORBA_Bounded_Sequence<T,n>* p){
  s = S(size,S::allocbuf(size), 1);}
template<class T1, class T2> inline
void convert(T1& in, T2& out){ out = static_cast<T2>(in); }
template<typename T>
void convert(T& in, std::string& out){ out = std::string(in); }
template<typename T>
void convert(std::string& in, T& out){ out = static_cast<T>(in.c_str()); }
void convert(_CORBA_String_element in, std::string& out){ out = std::string(in); }
void convert(std::string& in, _CORBA_String_element out){ out = (const char*)in.c_str(); }
template<class S,class T>
void convert(S& s, std::vector<T>& v){
  int size = s.length();
  v = std::vector<T>(s.length());
  for(int i=0; i<size; i++) convert(s[i],v[i]);}
template<class S,class T>
void convert(std::vector<T>& v, S& s){
  int size = v.size();
  s = S(size, size, S::allocbuf(size), 1);
  for(int i=0; i<size; i++) convert(v[i],s[i]);}
template<class S,class T,std::size_t n>
void convert(S& s, boost::array<T,n>& v){
  for(std::size_t i=0; i<n; i++) convert(s[i],v[i]);}
template<class S,class T,std::size_t n>
void convert(boost::array<T,n>& v, S& s){
  s = S(n, S::allocbuf(n), 1);
  for(std::size_t i=0; i<n; i++) convert(v[i],s[i]);}
template<typename S,class T,std::size_t n>
void convert(boost::array<T,n>& v, S (&s)[n]){
  for(std::size_t i=0; i<n; i++) convert(v[i],s[i]);}

// special case for RTC::LightweightRTObject_var
template<class T> void convert(T& in, RTC::LightweightRTObject_var out){ std::cerr << "convert from RTC::LightweightRTObject_var is not supported" << std::endl; }
template<> void convert(whole_body_master_slave_controller::EEFFrameConverterService::EEFFrameConverterParam& in, whole_body_master_slave_controller::whole_body_master_slave_controller_EEFFrameConverterService_EEFFrameConverterParam& out){
  convert(in.debugLevel, out.debugLevel);
}
template<> void convert(whole_body_master_slave_controller::whole_body_master_slave_controller_EEFFrameConverterService_EEFFrameConverterParam& in, whole_body_master_slave_controller::EEFFrameConverterService::EEFFrameConverterParam& out){
  convert(in.debugLevel, out.debugLevel);
}
bool EEFFrameConverterServiceROSBridge::setParams(whole_body_master_slave_controller::whole_body_master_slave_controller_EEFFrameConverterService_setParams::Request &req, whole_body_master_slave_controller::whole_body_master_slave_controller_EEFFrameConverterService_setParams::Response &res){
  whole_body_master_slave_controller::EEFFrameConverterService::EEFFrameConverterParam i_param;

  ROS_INFO_STREAM("EEFFrameConverterServiceROSBridge::setParams() called");

  convert(req.i_param, i_param);

  try {
    m_service0->setParams(i_param);
  } catch(CORBA::COMM_FAILURE& ex) {
      ROS_ERROR_STREAM("EEFFrameConverterServiceROSBridge::setParams : Caught system exception COMM_FAILURE -- unable to contact the object [minor code = " << ex.minor() << "]. One possibility is IDL version mismatch between EEFFrameConverterServiceROSBridge and EEFFrameConverter. So please check IDL versions are consistent. Another possibility is that Service-Port function, setParams of EEFFrameConverter, called but EEFFrameConverter died becase of it. So please check EEFFrameConverter is still alive.");
      return false;
  } catch(CORBA::MARSHAL& ex) {
      ROS_ERROR_STREAM("EEFFrameConverterServiceROSBridge::setParams : Caught CORBA::SystemException::MARSHAL [minor code = " << ex.minor() << "]. One possibility is IDL version mismatch between EEFFrameConverterServiceROSBridge and EEFFrameConverter. So please check IDL versions are consistent.");
      return false;
  } catch(CORBA::BAD_PARAM& ex) {
      ROS_ERROR_STREAM("EEFFrameConverterServiceROSBridge::setParams : Caught CORBA::SystemException::BAD_PARAM [minor code = " << ex.minor() << "]. One possibility is that Service-Port function, setParams of EEFFrameConverter, called and some problem has occurred within it and EEFFrameConverter keeps alive. So please check setParams of EEFFrameConverter is correctly finished.");
      return false;
  } catch(CORBA::OBJECT_NOT_EXIST& ex) {
      ROS_ERROR_STREAM("EEFFrameConverterServiceROSBridge::setParams : Caught CORBA::SystemException::OBJECT_NOT_EXIST [minor code = " << ex.minor() << "]. One possibility is that EEFFrameConverter RTC is not found. So please check EEFFrameConverter is still alive.");
      return false;
  } catch(CORBA::SystemException& ex) {
      ROS_ERROR_STREAM("EEFFrameConverterServiceROSBridge::setParams : Caught CORBA::SystemException [minor code = " << ex.minor() << "].");
      return false;
  } catch(CORBA::Exception&) {
      ROS_ERROR_STREAM("EEFFrameConverterServiceROSBridge::setParams : Caught CORBA::Exception.");
      return false;
  } catch(omniORB::fatalException& fe) {
      ROS_ERROR_STREAM("EEFFrameConverterServiceROSBridge::setParams : Caught omniORB::fatalException:");
      ROS_ERROR_STREAM("  file: " << fe.file());
      ROS_ERROR_STREAM("  line: " << fe.line());
      ROS_ERROR_STREAM("  mesg: " << fe.errmsg());
      return false;
  }
  catch(...) {
      ROS_ERROR_STREAM("EEFFrameConverterServiceROSBridge::setParams : Caught unknown exception.");
      return false;
  }
  ROS_INFO_STREAM("EEFFrameConverterServiceROSBridge::setParams() succeeded");

  return true;
};

bool EEFFrameConverterServiceROSBridge::getParams(whole_body_master_slave_controller::whole_body_master_slave_controller_EEFFrameConverterService_getParams::Request &req, whole_body_master_slave_controller::whole_body_master_slave_controller_EEFFrameConverterService_getParams::Response &res){
  whole_body_master_slave_controller::EEFFrameConverterService::EEFFrameConverterParam i_param;

  ROS_INFO_STREAM("EEFFrameConverterServiceROSBridge::getParams() called");


  try {
    m_service0->getParams(i_param);
  } catch(CORBA::COMM_FAILURE& ex) {
      ROS_ERROR_STREAM("EEFFrameConverterServiceROSBridge::getParams : Caught system exception COMM_FAILURE -- unable to contact the object [minor code = " << ex.minor() << "]. One possibility is IDL version mismatch between EEFFrameConverterServiceROSBridge and EEFFrameConverter. So please check IDL versions are consistent. Another possibility is that Service-Port function, getParams of EEFFrameConverter, called but EEFFrameConverter died becase of it. So please check EEFFrameConverter is still alive.");
      return false;
  } catch(CORBA::MARSHAL& ex) {
      ROS_ERROR_STREAM("EEFFrameConverterServiceROSBridge::getParams : Caught CORBA::SystemException::MARSHAL [minor code = " << ex.minor() << "]. One possibility is IDL version mismatch between EEFFrameConverterServiceROSBridge and EEFFrameConverter. So please check IDL versions are consistent.");
      return false;
  } catch(CORBA::BAD_PARAM& ex) {
      ROS_ERROR_STREAM("EEFFrameConverterServiceROSBridge::getParams : Caught CORBA::SystemException::BAD_PARAM [minor code = " << ex.minor() << "]. One possibility is that Service-Port function, getParams of EEFFrameConverter, called and some problem has occurred within it and EEFFrameConverter keeps alive. So please check getParams of EEFFrameConverter is correctly finished.");
      return false;
  } catch(CORBA::OBJECT_NOT_EXIST& ex) {
      ROS_ERROR_STREAM("EEFFrameConverterServiceROSBridge::getParams : Caught CORBA::SystemException::OBJECT_NOT_EXIST [minor code = " << ex.minor() << "]. One possibility is that EEFFrameConverter RTC is not found. So please check EEFFrameConverter is still alive.");
      return false;
  } catch(CORBA::SystemException& ex) {
      ROS_ERROR_STREAM("EEFFrameConverterServiceROSBridge::getParams : Caught CORBA::SystemException [minor code = " << ex.minor() << "].");
      return false;
  } catch(CORBA::Exception&) {
      ROS_ERROR_STREAM("EEFFrameConverterServiceROSBridge::getParams : Caught CORBA::Exception.");
      return false;
  } catch(omniORB::fatalException& fe) {
      ROS_ERROR_STREAM("EEFFrameConverterServiceROSBridge::getParams : Caught omniORB::fatalException:");
      ROS_ERROR_STREAM("  file: " << fe.file());
      ROS_ERROR_STREAM("  line: " << fe.line());
      ROS_ERROR_STREAM("  mesg: " << fe.errmsg());
      return false;
  }
  catch(...) {
      ROS_ERROR_STREAM("EEFFrameConverterServiceROSBridge::getParams : Caught unknown exception.");
      return false;
  }
  convert(i_param, res.i_param);
  ROS_INFO_STREAM("EEFFrameConverterServiceROSBridge::getParams() succeeded");

  return true;
};

bool EEFFrameConverterServiceROSBridge::startControl(whole_body_master_slave_controller::whole_body_master_slave_controller_EEFFrameConverterService_startControl::Request &req, whole_body_master_slave_controller::whole_body_master_slave_controller_EEFFrameConverterService_startControl::Response &res){

  ROS_INFO_STREAM("EEFFrameConverterServiceROSBridge::startControl() called");


  try {
    res.operation_return = m_service0->startControl();
  } catch(CORBA::COMM_FAILURE& ex) {
      ROS_ERROR_STREAM("EEFFrameConverterServiceROSBridge::startControl : Caught system exception COMM_FAILURE -- unable to contact the object [minor code = " << ex.minor() << "]. One possibility is IDL version mismatch between EEFFrameConverterServiceROSBridge and EEFFrameConverter. So please check IDL versions are consistent. Another possibility is that Service-Port function, startControl of EEFFrameConverter, called but EEFFrameConverter died becase of it. So please check EEFFrameConverter is still alive.");
      return false;
  } catch(CORBA::MARSHAL& ex) {
      ROS_ERROR_STREAM("EEFFrameConverterServiceROSBridge::startControl : Caught CORBA::SystemException::MARSHAL [minor code = " << ex.minor() << "]. One possibility is IDL version mismatch between EEFFrameConverterServiceROSBridge and EEFFrameConverter. So please check IDL versions are consistent.");
      return false;
  } catch(CORBA::BAD_PARAM& ex) {
      ROS_ERROR_STREAM("EEFFrameConverterServiceROSBridge::startControl : Caught CORBA::SystemException::BAD_PARAM [minor code = " << ex.minor() << "]. One possibility is that Service-Port function, startControl of EEFFrameConverter, called and some problem has occurred within it and EEFFrameConverter keeps alive. So please check startControl of EEFFrameConverter is correctly finished.");
      return false;
  } catch(CORBA::OBJECT_NOT_EXIST& ex) {
      ROS_ERROR_STREAM("EEFFrameConverterServiceROSBridge::startControl : Caught CORBA::SystemException::OBJECT_NOT_EXIST [minor code = " << ex.minor() << "]. One possibility is that EEFFrameConverter RTC is not found. So please check EEFFrameConverter is still alive.");
      return false;
  } catch(CORBA::SystemException& ex) {
      ROS_ERROR_STREAM("EEFFrameConverterServiceROSBridge::startControl : Caught CORBA::SystemException [minor code = " << ex.minor() << "].");
      return false;
  } catch(CORBA::Exception&) {
      ROS_ERROR_STREAM("EEFFrameConverterServiceROSBridge::startControl : Caught CORBA::Exception.");
      return false;
  } catch(omniORB::fatalException& fe) {
      ROS_ERROR_STREAM("EEFFrameConverterServiceROSBridge::startControl : Caught omniORB::fatalException:");
      ROS_ERROR_STREAM("  file: " << fe.file());
      ROS_ERROR_STREAM("  line: " << fe.line());
      ROS_ERROR_STREAM("  mesg: " << fe.errmsg());
      return false;
  }
  catch(...) {
      ROS_ERROR_STREAM("EEFFrameConverterServiceROSBridge::startControl : Caught unknown exception.");
      return false;
  }
  ROS_INFO_STREAM("EEFFrameConverterServiceROSBridge::startControl() succeeded");

  return true;
};

bool EEFFrameConverterServiceROSBridge::stopControl(whole_body_master_slave_controller::whole_body_master_slave_controller_EEFFrameConverterService_stopControl::Request &req, whole_body_master_slave_controller::whole_body_master_slave_controller_EEFFrameConverterService_stopControl::Response &res){

  ROS_INFO_STREAM("EEFFrameConverterServiceROSBridge::stopControl() called");


  try {
    res.operation_return = m_service0->stopControl();
  } catch(CORBA::COMM_FAILURE& ex) {
      ROS_ERROR_STREAM("EEFFrameConverterServiceROSBridge::stopControl : Caught system exception COMM_FAILURE -- unable to contact the object [minor code = " << ex.minor() << "]. One possibility is IDL version mismatch between EEFFrameConverterServiceROSBridge and EEFFrameConverter. So please check IDL versions are consistent. Another possibility is that Service-Port function, stopControl of EEFFrameConverter, called but EEFFrameConverter died becase of it. So please check EEFFrameConverter is still alive.");
      return false;
  } catch(CORBA::MARSHAL& ex) {
      ROS_ERROR_STREAM("EEFFrameConverterServiceROSBridge::stopControl : Caught CORBA::SystemException::MARSHAL [minor code = " << ex.minor() << "]. One possibility is IDL version mismatch between EEFFrameConverterServiceROSBridge and EEFFrameConverter. So please check IDL versions are consistent.");
      return false;
  } catch(CORBA::BAD_PARAM& ex) {
      ROS_ERROR_STREAM("EEFFrameConverterServiceROSBridge::stopControl : Caught CORBA::SystemException::BAD_PARAM [minor code = " << ex.minor() << "]. One possibility is that Service-Port function, stopControl of EEFFrameConverter, called and some problem has occurred within it and EEFFrameConverter keeps alive. So please check stopControl of EEFFrameConverter is correctly finished.");
      return false;
  } catch(CORBA::OBJECT_NOT_EXIST& ex) {
      ROS_ERROR_STREAM("EEFFrameConverterServiceROSBridge::stopControl : Caught CORBA::SystemException::OBJECT_NOT_EXIST [minor code = " << ex.minor() << "]. One possibility is that EEFFrameConverter RTC is not found. So please check EEFFrameConverter is still alive.");
      return false;
  } catch(CORBA::SystemException& ex) {
      ROS_ERROR_STREAM("EEFFrameConverterServiceROSBridge::stopControl : Caught CORBA::SystemException [minor code = " << ex.minor() << "].");
      return false;
  } catch(CORBA::Exception&) {
      ROS_ERROR_STREAM("EEFFrameConverterServiceROSBridge::stopControl : Caught CORBA::Exception.");
      return false;
  } catch(omniORB::fatalException& fe) {
      ROS_ERROR_STREAM("EEFFrameConverterServiceROSBridge::stopControl : Caught omniORB::fatalException:");
      ROS_ERROR_STREAM("  file: " << fe.file());
      ROS_ERROR_STREAM("  line: " << fe.line());
      ROS_ERROR_STREAM("  mesg: " << fe.errmsg());
      return false;
  }
  catch(...) {
      ROS_ERROR_STREAM("EEFFrameConverterServiceROSBridge::stopControl : Caught unknown exception.");
      return false;
  }
  ROS_INFO_STREAM("EEFFrameConverterServiceROSBridge::stopControl() succeeded");

  return true;
};

