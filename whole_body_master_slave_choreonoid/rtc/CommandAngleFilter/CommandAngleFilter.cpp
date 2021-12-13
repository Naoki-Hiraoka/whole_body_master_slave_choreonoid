#include "CommandAngleFilter.h"
#include <cnoid/EigenUtil>
#include <cnoid/BodyLoader>
#include <cnoid/BasicSensors>
#include <cnoid/ValueTree>

#define DEBUGP (loop%200==0)
#define DEBUGP_ONCE (loop==0)

static const char* CommandAngleFilter_spec[] = {
  "implementation_id", "CommandAngleFilter",
  "type_name",         "CommandAngleFilter",
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

CommandAngleFilter::CommandAngleFilter(RTC::Manager* manager) : RTC::DataFlowComponentBase(manager),
  ports_(),
  debugLevel_(0)
{
  this->ports_.m_service0_.setComp(this);
}

RTC::ReturnCode_t CommandAngleFilter::onInitialize(){

  addInPort("qIn", this->ports_.m_qIn_);
  addOutPort("qOut", this->ports_.m_qOut_);
  addInPort("basePoseIn", this->ports_.m_basePoseIn_);
  addOutPort("basePoseOut", this->ports_.m_basePoseOut_);
  this->ports_.m_CommandAngleFilterServicePort_.registerProvider("service0", "CommandAngleFilterService", this->ports_.m_service0_);
  addPort(this->ports_.m_CommandAngleFilterServicePort_);

  cnoid::BodyLoader bodyLoader;
  std::string fileName;
  if(this->getProperties().hasKey("model")) fileName = std::string(this->getProperties()["model"]);
  else fileName = std::string(this->m_pManager->getConfig()["model"]); // 引数 -o で与えたプロパティを捕捉
  std::cerr << "[" << this->m_profile.instance_name << "] model: " << fileName <<std::endl;
  this->robot_ref_ = bodyLoader.load(fileName);
  if(!this->robot_ref_){
    std::cerr << "\x1b[31m[" << m_profile.instance_name << "] " << "failed to load model[" << fileName << "]" << "\x1b[39m" << std::endl;
    return RTC::RTC_ERROR;
  }
  this->robot_com_ = this->robot_ref_->clone();

  {
    std::string minGoalTimeStr = "0.0";
    if(this->getProperties().hasKey("min_goal_time")) minGoalTimeStr = std::string(this->getProperties()["min_goal_time"]);
    else if(this->m_pManager->getConfig().hasKey("min_goal_time")) minGoalTimeStr = std::string(this->m_pManager->getConfig()["min_goal_time"]); // 引数 -o で与えたプロパティを捕捉
    std::cerr << "[" << this->m_profile.instance_name << "] min_goal_time: " << minGoalTimeStr <<std::endl;
    this->minGoalTime_ = std::stod(minGoalTimeStr);
  }

  // load joint limit table
  std::string jointLimitTableProp;
  if(this->getProperties().hasKey("joint_limit_table")) jointLimitTableProp = std::string(this->getProperties()["joint_limit_table"]);
  else jointLimitTableProp = std::string(this->m_pManager->getConfig()["joint_limit_table"]); // 引数 -o で与えたプロパティを捕捉
  std::cerr << "[" << this->m_profile.instance_name << "] joint_limit_table: " << jointLimitTableProp<<std::endl;
  std::vector<std::shared_ptr<joint_limit_table::JointLimitTable> > jointLimitTablesVec
    = joint_limit_table::readJointLimitTablesFromProperty (this->robot_com_, jointLimitTableProp);
  for(size_t i=0;i<jointLimitTablesVec.size();i++){
    this->joint_limit_tables_[jointLimitTablesVec[i]->getSelfJoint()->name()].push_back(jointLimitTablesVec[i]);
  }

  this->loop_ = 0;

  this->qInterpolator_ = std::make_shared<cpp_filters::TwoPointInterpolator<cnoid::VectorX> >(cnoid::VectorX::Zero(this->robot_com_->numJoints()), cnoid::VectorX::Zero(this->robot_com_->numJoints()), cnoid::VectorX::Zero(this->robot_com_->numJoints()), cpp_filters::CUBICSPLINE);
  this->qOffsetInterpolator_ = std::make_shared<cpp_filters::TwoPointInterpolator<cnoid::VectorX> >(cnoid::VectorX::Zero(this->robot_com_->numJoints()), cnoid::VectorX::Zero(this->robot_com_->numJoints()), cnoid::VectorX::Zero(this->robot_com_->numJoints()), cpp_filters::HOFFARBIB);
  this->rootpInterpolator_= std::make_shared<cpp_filters::TwoPointInterpolator<cnoid::Vector3> >(cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cpp_filters::CUBICSPLINE);
  this->rootpOffsetInterpolator_= std::make_shared<cpp_filters::TwoPointInterpolator<cnoid::Vector3> >(cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cpp_filters::HOFFARBIB);
  this->rootRInterpolator_= std::make_shared<cpp_filters::TwoPointInterpolatorSO3 >(cnoid::Matrix3::Identity(),cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cpp_filters::CUBICSPLINE);
  this->rootROffsetInterpolator_= std::make_shared<cpp_filters::TwoPointInterpolatorSO3 >(cnoid::Matrix3::Identity(),cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cpp_filters::HOFFARBIB);

  return RTC::RTC_OK;
}

RTC::ReturnCode_t CommandAngleFilter::onExecute(RTC::UniqueId ec_id){
  std::lock_guard<std::mutex> guard(this->mutex_);

  std::string instance_name = std::string(this->m_profile.instance_name);
  double dt = 1.0 / this->get_context(ec_id)->get_rate();

  // read ports
  if(this->ports_.m_qIn_.isNew()){
    this->ports_.m_qIn_.read();
    if(this->ports_.m_q_.data.length() == this->robot_ref_->numJoints()){
      for(int i=0;i<this->robot_ref_->numJoints();i++){
        this->robot_ref_->joint(i)->q() = this->ports_.m_q_.data[i];
      }
    }
  }
  if(this->ports_.m_basePoseIn_.isNew()) {
    this->ports_.m_basePoseIn_.read();
    this->robot_ref_->rootLink()->p()[0] = this->ports_.m_basePose_.data.position.x;
    this->robot_ref_->rootLink()->p()[1] = this->ports_.m_basePose_.data.position.y;
    this->robot_ref_->rootLink()->p()[2] = this->ports_.m_basePose_.data.position.z;
    this->robot_ref_->rootLink()->R() = cnoid::rotFromRpy(this->ports_.m_basePose_.data.orientation.r,
                                                          this->ports_.m_basePose_.data.orientation.p,
                                                          this->ports_.m_basePose_.data.orientation.y);
  }
  cnoid::VectorX qref = cnoid::VectorX::Zero(this->robot_ref_->numJoints());
  for(int i=0;i<this->robot_ref_->numJoints();i++) qref[i] = this->robot_ref_->joint(i)->q();

  // mode transition
  switch(this->mode_.now()){
  case CommandAngleFilter::ControlMode::MODE_SYNC_TO_CONTROL:
    this->mode_.setNextMode(CommandAngleFilter::ControlMode::MODE_CONTROL);
    break;
  case CommandAngleFilter::ControlMode::MODE_SYNC_TO_IDLE:
    if(this->mode_.pre() == CommandAngleFilter::ControlMode::MODE_CONTROL){
      cnoid::VectorX qout = cnoid::VectorX::Zero(this->robot_ref_->numJoints());
      this->qInterpolator_->get(qout);
      this->qOffsetInterpolator_->reset(qout - qref);
      this->qOffsetInterpolator_->setGoal(cnoid::VectorX::Zero(this->robot_ref_->numJoints()), 3.0);
      cnoid::Vector3 rootpout = cnoid::Vector3::Zero();
      this->rootpInterpolator_->get(rootpout);
      this->rootpOffsetInterpolator_->reset(rootpout - this->robot_ref_->rootLink()->p());
      this->rootpOffsetInterpolator_->setGoal(cnoid::Vector3::Zero(), 3.0);
      cnoid::Matrix3 rootRout = cnoid::Matrix3::Identity();
      this->rootRInterpolator_->get(rootRout);
      this->rootROffsetInterpolator_->reset(rootRout * this->robot_ref_->rootLink()->R().transpose());
      this->rootROffsetInterpolator_->setGoal(cnoid::Matrix3::Identity(), 3.0);
    }
    this->mode_.setNextMode(CommandAngleFilter::ControlMode::MODE_IDLE);
    break;
  }
  this->mode_.update();

  cnoid::VectorX qout = cnoid::VectorX::Zero(this->robot_ref_->numJoints());
  cnoid::VectorX dqout = cnoid::VectorX::Zero(this->robot_ref_->numJoints());
  cnoid::Vector3 rootpout = cnoid::Vector3::Zero();
  cnoid::Vector3 rootvout = cnoid::Vector3::Zero();
  cnoid::Matrix3 rootRout = cnoid::Matrix3::Identity();
  cnoid::Vector3 rootwout = cnoid::Vector3::Zero();
  if(this->mode_.isRunning()) {

    // set goal
    this->qInterpolator_->setGoal(qref, std::max(this->minGoalTime_, dt));
    this->rootpInterpolator_->setGoal(this->robot_ref_->rootLink()->p(), std::max(this->minGoalTime_, dt));
    this->rootRInterpolator_->setGoal(this->robot_ref_->rootLink()->R(), std::max(this->minGoalTime_, dt));

    // get next command
    this->qInterpolator_->get(qout, dqout, dt);
    this->rootpInterpolator_->get(rootpout, rootvout, dt);
    this->rootRInterpolator_->get(rootRout, rootwout, dt);

    // apply limit
    for(int i=0;i<this->robot_com_->numJoints();i++) {
      double llimit = this->robot_com_->joint(i)->q_lower();
      double ulimit = this->robot_com_->joint(i)->q_upper();
      if (this->joint_limit_tables_.find(this->robot_com_->joint(i)->name()) != this->joint_limit_tables_.end()) {
        std::vector<std::shared_ptr<joint_limit_table::JointLimitTable> >& tables = this->joint_limit_tables_[this->robot_com_->joint(i)->name()];
        for(int j=0;j<tables.size();j++){
          llimit = std::max(llimit, tables[j]->getLlimit());
          ulimit = std::min(ulimit, tables[j]->getUlimit());
        }
      }
      qout[i] = std::min(ulimit, std::max(llimit, qout[i]));
    }

  } else {

    cnoid::VectorX qoffset = cnoid::VectorX::Zero(this->robot_ref_->numJoints());
    if(!this->qOffsetInterpolator_->isEmpty()) this->qOffsetInterpolator_->get(qoffset, dt);
    qout = qref + qoffset;

    cnoid::Vector3 rootpoffset = cnoid::Vector3::Zero();
    if(!this->rootpOffsetInterpolator_->isEmpty()) this->rootpOffsetInterpolator_->get(rootpoffset, dt);
    rootpout = this->robot_ref_->rootLink()->p() + rootpoffset;

    cnoid::Matrix3 rootRoffset = cnoid::Matrix3::Identity();
    if(!this->rootROffsetInterpolator_->isEmpty()) this->rootROffsetInterpolator_->get(rootRoffset, dt);
    rootRout = rootRoffset * this->robot_ref_->rootLink()->R();
  }

  // set robot_com
  // and
  // reinitialize interpolator
  for(int i=0;i<this->robot_com_->numJoints();i++) this->robot_com_->joint(i)->q() = qout[i];
  this->qInterpolator_->reset(qout, dqout);
  this->robot_com_->rootLink()->p() = rootpout;
  this->rootpInterpolator_->reset(rootpout, rootvout);
  this->robot_com_->rootLink()->R() = rootRout;
  this->rootRInterpolator_->reset(rootRout, rootwout);

  // output ports
  this->ports_.m_qCom_.tm = this->ports_.m_q_.tm;
  this->ports_.m_qCom_.data.length(this->robot_com_->numJoints());
  for(int i=0;i<this->robot_com_->numJoints();i++){
    this->ports_.m_qCom_.data[i] = this->robot_com_->joint(i)->q();
  }
  this->ports_.m_qOut_.write();

  this->ports_.m_basePoseCom_.tm = this->ports_.m_basePose_.tm;
  this->ports_.m_basePoseCom_.data.position.x = this->robot_com_->rootLink()->p()[0];
  this->ports_.m_basePoseCom_.data.position.y = this->robot_com_->rootLink()->p()[1];
  this->ports_.m_basePoseCom_.data.position.z = this->robot_com_->rootLink()->p()[2];
  cnoid::Vector3 outputBaseRpy = cnoid::rpyFromRot(this->robot_com_->rootLink()->R());
  this->ports_.m_basePoseCom_.data.orientation.r = outputBaseRpy[0];
  this->ports_.m_basePoseCom_.data.orientation.p = outputBaseRpy[1];
  this->ports_.m_basePoseCom_.data.orientation.y = outputBaseRpy[2];
  this->ports_.m_basePoseOut_.write();

  this->loop_++;
  return RTC::RTC_OK;
}

bool CommandAngleFilter::setParams(const whole_body_master_slave_choreonoid::CommandAngleFilterService::CommandAngleFilterParam& i_param){
  std::lock_guard<std::mutex> guard(this->mutex_);
  std::cerr << "[" << m_profile.instance_name << "] "<< "setParams" << std::endl;
  this->debugLevel_ = i_param.debugLevel;
  this->minGoalTime_ = std::max(i_param.minGoalTime, 0.0);

  return true;
}


bool CommandAngleFilter::getParams(whole_body_master_slave_choreonoid::CommandAngleFilterService::CommandAngleFilterParam& i_param){
  std::cerr << "[" << m_profile.instance_name << "] "<< "getParams" << std::endl;
  i_param.debugLevel = this->debugLevel_;
  i_param.minGoalTime = this->minGoalTime_;
  return true;
}

bool CommandAngleFilter::startControl(){
  std::lock_guard<std::mutex> guard(this->mutex_);
  if(this->mode_.now() == ControlMode::MODE_IDLE){
    std::cerr << "[" << m_profile.instance_name << "] "<< "startControl" << std::endl;
    this->mode_.setNextMode(ControlMode::MODE_SYNC_TO_CONTROL);
    return true;
  }else{
    std::cerr << "\x1b[31m[" << m_profile.instance_name << "] " << "Invalid context to startControl" << "\x1b[39m" << std::endl;
    return false;
  }
}


bool CommandAngleFilter::stopControl(){
  std::lock_guard<std::mutex> guard(this->mutex_);
  if(this->mode_.now() == ControlMode::MODE_CONTROL ){
    std::cerr << "[" << m_profile.instance_name << "] "<< "stopControl" << std::endl;
    this->mode_.setNextMode(ControlMode::MODE_SYNC_TO_IDLE);
    return true;
  }else{
    std::cerr << "\x1b[31m[" << m_profile.instance_name << "] " << "Invalid context to stopControl" << "\x1b[39m" << std::endl;
    return false;
  }
}

RTC::ReturnCode_t CommandAngleFilter::onActivated(RTC::UniqueId ec_id){
  std::cerr << "[" << m_profile.instance_name << "] "<< "onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}
RTC::ReturnCode_t CommandAngleFilter::onDeactivated(RTC::UniqueId ec_id){
  std::cerr << "[" << m_profile.instance_name << "] "<< "onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}
RTC::ReturnCode_t CommandAngleFilter::onFinalize(){ return RTC::RTC_OK; }

extern "C"{
    void CommandAngleFilterInit(RTC::Manager* manager) {
        RTC::Properties profile(CommandAngleFilter_spec);
        manager->registerFactory(profile, RTC::Create<CommandAngleFilter>, RTC::Delete<CommandAngleFilter>);
    }
};
