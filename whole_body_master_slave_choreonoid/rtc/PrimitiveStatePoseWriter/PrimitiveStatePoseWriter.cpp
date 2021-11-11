#include "PrimitiveStatePoseWriter.h"
#include <cnoid/EigenUtil>
#include <cnoid/BodyLoader>
#include <cnoid/BasicSensors>
#include <cnoid/ValueTree>

#define DEBUGP (loop%200==0)
#define DEBUGP_ONCE (loop==0)

static const char* PrimitiveStatePoseWriter_spec[] = {
  "implementation_id", "PrimitiveStatePoseWriter",
  "type_name",         "PrimitiveStatePoseWriter",
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

PrimitiveStatePoseWriter::PrimitiveStatePoseWriter(RTC::Manager* manager) : RTC::DataFlowComponentBase(manager),
  ports_()
{
}

RTC::ReturnCode_t PrimitiveStatePoseWriter::onInitialize(){

  addInPort("primitiveStateRefIn", this->ports_.m_primitiveStateRefIn_);
  addInPort("qIn", this->ports_.m_qIn_);
  addInPort("basePoseIn", this->ports_.m_basePoseIn_);
  addOutPort("primitiveStateComOut", this->ports_.m_primitiveStateComOut_);

  cnoid::BodyLoader bodyLoader;
  std::string fileName;
  if(this->getProperties().hasKey("model")) fileName = std::string(this->getProperties()["model"]);
  else fileName = std::string(this->m_pManager->getConfig()["model"]); // 引数 -o で与えたプロパティを捕捉
  std::cerr << "[" << this->m_profile.instance_name << "] model: " << fileName <<std::endl;
  this->robot_ = bodyLoader.load(fileName);
  if(!this->robot_){
    std::cerr << "\x1b[31m[" << m_profile.instance_name << "] " << "failed to load model[" << fileName << "]" << "\x1b[39m" << std::endl;
    return RTC::RTC_ERROR;
  }

  this->loop_ = 0;

  return RTC::RTC_OK;
}

void PrimitiveStatePoseWriter::getPrimitiveState(const std::string& instance_name, PrimitiveStatePoseWriter::Ports& port, double dt, primitive_motion_level_tools::PrimitiveStates& primitiveStates) {
  if(port.m_primitiveStateRefIn_.isNew()) {
    port.m_primitiveStateRefIn_.read();
    primitiveStates.updateFromIdl(port.m_primitiveStateRef_);
  }

  // 補間をdtすすめる
  primitiveStates.updateTargetForOneStep(dt);
}

void PrimitiveStatePoseWriter::getRobot(const std::string& instance_name, PrimitiveStatePoseWriter::Ports& port, cnoid::BodyPtr& robot) {
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

void PrimitiveStatePoseWriter::calcOutputPorts(const std::string& instance_name,
                                               const primitive_motion_level_tools::PrimitiveStates& primitiveStates,
                                               const cnoid::BodyPtr& robot,
                                               double dt,
                                               PrimitiveStatePoseWriter::Ports& port){
  // primitiveState
  port.m_primitiveStateCom_ = port.m_primitiveStateRef_;
  for(int i=0;i<port.m_primitiveStateCom_.data.length();i++){
    std::string name = std::string(port.m_primitiveStateCom_.data[i].name);
    std::shared_ptr<primitive_motion_level_tools::PrimitiveState> primitiveState = primitiveStates.primitiveState().find(name)->second;
    if(port.m_primitiveStateCom_.data[i].time != 0.0) port.m_primitiveStateCom_.data[i].time = dt;
    const cnoid::Vector6& targetWrench = primitiveState->targetWrench();
    for(size_t j=0;j<6;j++) port.m_primitiveStateCom_.data[i].wrench[j] = targetWrench[j];

    cnoid::Position targetPose;
    cnoid::LinkPtr link = robot->link(primitiveState->parentLinkName());
    if(link){
      targetPose = link->T() * primitiveState->localPose();
    }else{
      targetPose = primitiveStates.primitiveState().find(name)->second->targetPose();
    }
    port.m_primitiveStateCom_.data[i].pose.position.x = targetPose.translation()[0];
    port.m_primitiveStateCom_.data[i].pose.position.y = targetPose.translation()[1];
    port.m_primitiveStateCom_.data[i].pose.position.z = targetPose.translation()[2];
    cnoid::Vector3 rpy = cnoid::rpyFromRot(targetPose.linear());
    port.m_primitiveStateCom_.data[i].pose.orientation.r = rpy[0];
    port.m_primitiveStateCom_.data[i].pose.orientation.p = rpy[1];
    port.m_primitiveStateCom_.data[i].pose.orientation.y = rpy[2];
  }
  port.m_primitiveStateComOut_.write();
}

RTC::ReturnCode_t PrimitiveStatePoseWriter::onExecute(RTC::UniqueId ec_id){
  std::lock_guard<std::mutex> guard(this->mutex_);

  std::string instance_name = std::string(this->m_profile.instance_name);
  double dt = 1.0 / this->get_context(ec_id)->get_rate();
  // read ports
  PrimitiveStatePoseWriter::getPrimitiveState(instance_name, this->ports_, dt, this->primitiveStates_);
  PrimitiveStatePoseWriter::getRobot(instance_name, this->ports_, this->robot_);

  // write outport
  PrimitiveStatePoseWriter::calcOutputPorts(instance_name, this->primitiveStates_, this->robot_, dt, this->ports_);

  this->loop_++;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t PrimitiveStatePoseWriter::onActivated(RTC::UniqueId ec_id){
  std::cerr << "[" << m_profile.instance_name << "] "<< "onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}
RTC::ReturnCode_t PrimitiveStatePoseWriter::onDeactivated(RTC::UniqueId ec_id){
  std::cerr << "[" << m_profile.instance_name << "] "<< "onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}
RTC::ReturnCode_t PrimitiveStatePoseWriter::onFinalize(){ return RTC::RTC_OK; }

extern "C"{
    void PrimitiveStatePoseWriterInit(RTC::Manager* manager) {
        RTC::Properties profile(PrimitiveStatePoseWriter_spec);
        manager->registerFactory(profile, RTC::Create<PrimitiveStatePoseWriter>, RTC::Delete<PrimitiveStatePoseWriter>);
    }
};
