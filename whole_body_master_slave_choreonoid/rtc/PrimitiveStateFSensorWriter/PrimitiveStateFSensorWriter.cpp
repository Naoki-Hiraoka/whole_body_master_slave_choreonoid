#include "PrimitiveStateFSensorWriter.h"
#include <cnoid/EigenUtil>
#include <cnoid/BodyLoader>
#include <cnoid/BasicSensors>
#include <cnoid/ValueTree>

#define DEBUGP (loop%200==0)
#define DEBUGP_ONCE (loop==0)

static const char* PrimitiveStateFSensorWriter_spec[] = {
  "implementation_id", "PrimitiveStateFSensorWriter",
  "type_name",         "PrimitiveStateFSensorWriter",
  "description",       "wholebodymasterslave component",
  "version",           "0.0",
  "vendor",            "Naoki-Hiraoka",
  "category",          "example",
  "activity_type",     "DataFlowComponent",
  "max_instance",      "10",
  "language",          "C++",
  "lang_type",         "compile",
  ""
};

PrimitiveStateFSensorWriter::PrimitiveStateFSensorWriter(RTC::Manager* manager) : RTC::DataFlowComponentBase(manager),
  ports_(),
  debugLevel_(0)
{
  this->ports_.m_service0_.setComp(this);
}

RTC::ReturnCode_t PrimitiveStateFSensorWriter::onInitialize(){

  addInPort("primitiveStateRefIn", this->ports_.m_primitiveStateRefIn_);
  addInPort("qIn", this->ports_.m_qIn_);
  addInPort("basePoseIn", this->ports_.m_basePoseIn_);
  addOutPort("primitiveStateComOut", this->ports_.m_primitiveStateComOut_);
  this->ports_.m_PrimitiveStateFSensorWriterServicePort_.registerProvider("service0", "PrimitiveStateFSensorWriterService", this->ports_.m_service0_);
  addPort(this->ports_.m_PrimitiveStateFSensorWriterServicePort_);

  cnoid::BodyLoader bodyLoader;
  std::string fileName;
  if(this->getProperties().hasKey("model")) fileName = std::string(this->getProperties()["model"]);
  else fileName = std::string(this->m_pManager->getConfig()["model"]); // 引数 -o で与えたプロパティを捕捉
  std::cerr << "[" << this->m_profile.instance_name << "] model: " << fileName <<std::endl;
  this->robot_act_ = bodyLoader.load(fileName);
  if(!this->robot_act_){
    std::cerr << "\x1b[31m[" << m_profile.instance_name << "] " << "failed to load model[" << fileName << "]" << "\x1b[39m" << std::endl;
    return RTC::RTC_ERROR;
  }

  {
    std::string paired_sensorStr;
    if(this->getProperties().hasKey("paired_sensor")) paired_sensorStr = std::string(this->getProperties()["paired_sensor"]);
    else paired_sensorStr = std::string(this->m_pManager->getConfig()["paired_sensor"]); // 引数 -o で与えたプロパティを捕捉
    std::cerr << "[" << this->m_profile.instance_name << "] paired_sensor: " << paired_sensorStr <<std::endl;
    std::stringstream ss(paired_sensorStr);
    std::string item;
    while (std::getline(ss, item, ',')) {
      std::stringstream ss2(item);
      std::string eefName;
      if(!std::getline(ss2, eefName, ':')) continue;
      std::string fsensorName;
      if(!std::getline(ss2, fsensorName, ':')) continue;
      cnoid::ForceSensorPtr fsensor = this->robot_act_->findDevice<cnoid::ForceSensor>(fsensorName);
      if(!fsensor) continue;
      this->sensorMap_[eefName] = fsensor;
      std::cerr << "[" << m_profile.instance_name << "] " << eefName << " : " << fsensorName << std::endl;
    }
  }

  {
    cnoid::DeviceList<cnoid::ForceSensor> fsensors = this->robot_act_->devices<cnoid::ForceSensor>();
    this->ports_.m_wrenches_.resize(fsensors.size());
    this->ports_.m_wrenchesIn_.resize(fsensors.size());
    for(int i=0;i<fsensors.size();i++){
      this->ports_.m_wrenchesIn_[i] = std::make_shared<RTC::InPort<RTC::TimedDoubleSeq> >(fsensors[i]->name().c_str(),this->ports_.m_wrenches_[i]);
      addInPort(fsensors[i]->name().c_str(), *(this->ports_.m_wrenchesIn_[i]));
    }
  }

  {
    std::string writeToTargetWrenchStr = "false";
    if(this->getProperties().hasKey("write_to_target_wrench")) writeToTargetWrenchStr = std::string(this->getProperties()["write_to_target_wrench"]);
    else if(this->m_pManager->getConfig().hasKey("write_to_target_wrench")) writeToTargetWrenchStr = std::string(this->m_pManager->getConfig()["write_to_target_wrench"]); // 引数 -o で与えたプロパティを捕捉
    std::cerr << "[" << this->m_profile.instance_name << "] write_to_target_wrench: " << writeToTargetWrenchStr <<std::endl;
    std::istringstream is(writeToTargetWrenchStr);
    is >> std::boolalpha >> this->writeToTargetWrench_;
  }

  this->loop_ = 0;

  return RTC::RTC_OK;
}

void PrimitiveStateFSensorWriter::getForceSensorData(const std::string& instance_name, PrimitiveStateFSensorWriter::Ports& port, cnoid::BodyPtr& robot) {
  for(int i=0;i<port.m_wrenchesIn_.size();i++){
    if(port.m_wrenchesIn_[i]->isNew()){
      port.m_wrenchesIn_[i]->read();
      if(port.m_wrenches_[i].data.length() == 6) {
        cnoid::ForceSensorPtr sensor = robot->findDevice<cnoid::ForceSensor>(port.m_wrenchesIn_[i]->name());
        for(int j=0;j<6;j++){
          sensor->F()[j] = port.m_wrenches_[i].data[j];
        }
      }
    }
  }
}


void PrimitiveStateFSensorWriter::getPrimitiveState(const std::string& instance_name, PrimitiveStateFSensorWriter::Ports& port, double dt, primitive_motion_level_tools::PrimitiveStates& primitiveStates) {
  if(port.m_primitiveStateRefIn_.isNew()) {
    port.m_primitiveStateRefIn_.read();
    primitiveStates.updateFromIdl(port.m_primitiveStateRef_);
  }

  // 補間をdtすすめる
  primitiveStates.updateTargetForOneStep(dt);
}

void PrimitiveStateFSensorWriter::getActualRobot(const std::string& instance_name, PrimitiveStateFSensorWriter::Ports& port, cnoid::BodyPtr& robot) {
  bool updated = false;
  if(port.m_qIn_.isNew()){
    port.m_qIn_.read();
    if(port.m_q_.data.length() == robot->numJoints()){
      for ( int i = 0; i < robot->numJoints(); i++ ){
        robot->joint(i)->q() = port.m_q_.data[i];
        robot->joint(i)->dq() = 0.0;
        robot->joint(i)->ddq() = 0.0;
      }
      updated=true;
    }
  }

  if(port.m_basePoseIn_.isNew()){
    port.m_basePoseIn_.read();
    robot->rootLink()->p()[0] = port.m_basePose_.data.position.x;
    robot->rootLink()->p()[1] = port.m_basePose_.data.position.y;
    robot->rootLink()->p()[2] = port.m_basePose_.data.position.z;
    robot->rootLink()->R() = cnoid::rotFromRpy(port.m_basePose_.data.orientation.r, port.m_basePose_.data.orientation.p, port.m_basePose_.data.orientation.y);
    updated = true;
  }

  if(updated) robot->calcForwardKinematics();
}

void PrimitiveStateFSensorWriter::calcOutputPorts(const std::string& instance_name,
                                                  const std::unordered_map<std::string, cnoid::ForceSensorPtr>& sensorMap,
                                                  const primitive_motion_level_tools::PrimitiveStates& primitiveStates,
                                                  const cnoid::BodyPtr& robot,
                                                  bool writeToTargetWrench,
                                                  double dt,
                                                  PrimitiveStateFSensorWriter::Ports& port){
  // primitiveState
  port.m_primitiveStateCom_ = port.m_primitiveStateRef_;
  for(int i=0;i<port.m_primitiveStateCom_.data.length();i++){
    std::string name = std::string(port.m_primitiveStateCom_.data[i].name);
    if(port.m_primitiveStateCom_.data[i].time != 0.0) port.m_primitiveStateCom_.data[i].time = dt;
    const cnoid::Vector6& targetWrench = primitiveStates.primitiveState().find(name)->second->targetWrench();
    for(size_t j=0;j<6;j++) port.m_primitiveStateCom_.data[i].wrench[j] = targetWrench[j];
    cnoid::Position targetPose = primitiveStates.primitiveState().find(name)->second->targetPose();
    port.m_primitiveStateCom_.data[i].pose.position.x = targetPose.translation()[0];
    port.m_primitiveStateCom_.data[i].pose.position.y = targetPose.translation()[1];
    port.m_primitiveStateCom_.data[i].pose.position.z = targetPose.translation()[2];
    cnoid::Vector3 rpy = cnoid::rpyFromRot(targetPose.linear());
    port.m_primitiveStateCom_.data[i].pose.orientation.r = rpy[0];
    port.m_primitiveStateCom_.data[i].pose.orientation.p = rpy[1];
    port.m_primitiveStateCom_.data[i].pose.orientation.y = rpy[2];

    if(sensorMap.find(name) != sensorMap.end()){
      cnoid::ForceSensorPtr sensor = sensorMap.find(name)->second;
      const std::shared_ptr<primitive_motion_level_tools::PrimitiveState>& primitiveState = primitiveStates.primitiveState().find(name)->second;
      cnoid::Position sensorT = sensor->link()->T() * sensor->T_local();
      cnoid::Position eefT = robot->link(primitiveState->parentLinkName())->T() * primitiveState->localPose();
      cnoid::Position eef2SensorT = eefT.inverse() * sensorT;
      cnoid::Vector6 wrench;
      wrench.head<3>() = eef2SensorT.linear() * sensor->F().head<3>();
      wrench.tail<3>() = eef2SensorT.linear() * sensor->F().tail<3>();
      wrench.tail<3>() += eef2SensorT.translation().cross(wrench.head<3>());
      if(writeToTargetWrench){
        cnoid::Vector6 wrenchGlobal;
        wrenchGlobal.head<3>() = eefT.linear() * wrench.head<3>();
        wrenchGlobal.tail<3>() = eefT.linear() * wrench.tail<3>();
        for(int j=0;j<6;j++) port.m_primitiveStateCom_.data[i].wrench[j] = wrenchGlobal[j];
      }else{
        for(int j=0;j<6;j++) port.m_primitiveStateCom_.data[i].actWrench[j] = wrench[j];
      }
    }
  }
  port.m_primitiveStateComOut_.write();
}

RTC::ReturnCode_t PrimitiveStateFSensorWriter::onExecute(RTC::UniqueId ec_id){
  std::lock_guard<std::mutex> guard(this->mutex_);

  std::string instance_name = std::string(this->m_profile.instance_name);
  double dt = 1.0 / this->get_context(ec_id)->get_rate();
  // read ports
  PrimitiveStateFSensorWriter::getPrimitiveState(instance_name, this->ports_, dt, this->primitiveStates_);
  PrimitiveStateFSensorWriter::getActualRobot(instance_name, this->ports_, this->robot_act_);
  PrimitiveStateFSensorWriter::getForceSensorData(instance_name, this->ports_, this->robot_act_);
  // write outport
  PrimitiveStateFSensorWriter::calcOutputPorts(instance_name, this->sensorMap_, this->primitiveStates_, this->robot_act_, this->writeToTargetWrench_, dt, this->ports_);

  this->loop_++;
  return RTC::RTC_OK;
}

bool PrimitiveStateFSensorWriter::setParams(const whole_body_master_slave_choreonoid::PrimitiveStateFSensorWriterService::PrimitiveStateFSensorWriterParam& i_param){
  std::lock_guard<std::mutex> guard(this->mutex_);
  std::cerr << "[" << m_profile.instance_name << "] "<< "setParams" << std::endl;
  this->debugLevel_ = i_param.debugLevel;

  return true;
}


bool PrimitiveStateFSensorWriter::getParams(whole_body_master_slave_choreonoid::PrimitiveStateFSensorWriterService::PrimitiveStateFSensorWriterParam& i_param){
  std::cerr << "[" << m_profile.instance_name << "] "<< "getParams" << std::endl;
  i_param.debugLevel = this->debugLevel_;
  return true;
}

RTC::ReturnCode_t PrimitiveStateFSensorWriter::onActivated(RTC::UniqueId ec_id){
  std::cerr << "[" << m_profile.instance_name << "] "<< "onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}
RTC::ReturnCode_t PrimitiveStateFSensorWriter::onDeactivated(RTC::UniqueId ec_id){
  std::cerr << "[" << m_profile.instance_name << "] "<< "onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}
RTC::ReturnCode_t PrimitiveStateFSensorWriter::onFinalize(){ return RTC::RTC_OK; }

extern "C"{
    void PrimitiveStateFSensorWriterInit(RTC::Manager* manager) {
        RTC::Properties profile(PrimitiveStateFSensorWriter_spec);
        manager->registerFactory(profile, RTC::Create<PrimitiveStateFSensorWriter>, RTC::Delete<PrimitiveStateFSensorWriter>);
    }
};
