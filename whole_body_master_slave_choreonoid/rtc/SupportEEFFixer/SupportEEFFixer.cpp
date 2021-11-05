#include "SupportEEFFixer.h"
#include <cnoid/EigenUtil>
#include <cnoid/BodyLoader>
#include <cnoid/BasicSensors>

#define DEBUGP (loop%200==0)
#define DEBUGP_ONCE (loop==0)

static const char* SupportEEFFixer_spec[] = {
  "implementation_id", "SupportEEFFixer",
  "type_name",         "SupportEEFFixer",
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

SupportEEFFixer::SupportEEFFixer(RTC::Manager* manager) : RTC::DataFlowComponentBase(manager),
  ports_(),
  debugLevel_(0)
{
  this->ports_.m_service0_.setComp(this);
}

RTC::ReturnCode_t SupportEEFFixer::onInitialize(){

  addInPort("primitiveStateRefIn", this->ports_.m_primitiveStateRefIn_);
  addInPort("previewPrimitiveStateRefIn", this->ports_.m_previewPrimitiveStateRefIn_);
  addInPort("qComIn", this->ports_.m_qComIn_);
  addInPort("basePoseComIn", this->ports_.m_basePoseComIn_);
  addOutPort("primitiveStateComOut", this->ports_.m_primitiveStateComOut_);
  addOutPort("verticesOut", this->ports_.m_verticesOut_);
  addOutPort("previewVerticesOut", this->ports_.m_previewVerticesOut_);
  this->ports_.m_SupportEEFFixerServicePort_.registerProvider("service0", "SupportEEFFixerService", this->ports_.m_service0_);
  addPort(this->ports_.m_SupportEEFFixerServicePort_);

  cnoid::BodyLoader bodyLoader;
  std::string fileName;
  if(this->getProperties().hasKey("model")) fileName = std::string(this->getProperties()["model"]);
  else fileName = std::string(this->m_pManager->getConfig()["model"]); // 引数 -o で与えたプロパティを捕捉
  std::cerr << "[" << this->m_profile.instance_name << "] model: " << fileName <<std::endl;
  this->robot_com_ = bodyLoader.load(fileName);
  if(!this->robot_com_){
    std::cerr << "\x1b[31m[" << m_profile.instance_name << "] " << "failed to load model[" << fileName << "]" << "\x1b[39m" << std::endl;
    return RTC::RTC_ERROR;
  }

  this->loop_ = 0;

  this->mode_.setNextMode(ControlMode::MODE_IDLE);

  this->outputCOMOffsetInterpolator_ = std::make_shared<cpp_filters::TwoPointInterpolator<cnoid::Vector3> >(cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cpp_filters::HOFFARBIB);

  this->regionMargin_ = 0.02;

  return RTC::RTC_OK;
}

void SupportEEFFixer::getCommandRobot(const std::string& instance_name, SupportEEFFixer::Ports& port, cnoid::BodyPtr& robot) {
  bool updated = false;
  if(port.m_qComIn_.isNew()){
    port.m_qComIn_.read();
    if(port.m_qCom_.data.length() == robot->numJoints()){
      for ( int i = 0; i < robot->numJoints(); i++ ){
        robot->joint(i)->q() = port.m_qCom_.data[i];
      }
    }
    updated = true;
  }
  if(port.m_basePoseComIn_.isNew()){
    port.m_basePoseComIn_.read();
    robot->rootLink()->p()[0] = port.m_basePoseCom_.data.position.x;
    robot->rootLink()->p()[1] = port.m_basePoseCom_.data.position.y;
    robot->rootLink()->p()[2] = port.m_basePoseCom_.data.position.z;
    robot->rootLink()->R() = cnoid::rotFromRpy(port.m_basePoseCom_.data.orientation.r, port.m_basePoseCom_.data.orientation.p, port.m_basePoseCom_.data.orientation.y);
    updated = true;
  }
  if(updated){
    robot->calcForwardKinematics();
    robot->calcCenterOfMass();
  }
}


void SupportEEFFixer::getPrimitiveState(const std::string& instance_name, SupportEEFFixer::Ports& port, double dt, primitive_motion_level_tools::PrimitiveStates& primitiveStates) {
  if(port.m_primitiveStateRefIn_.isNew()) {
    port.m_primitiveStateRefIn_.read();
    primitiveStates.updateFromIdl(port.m_primitiveStateRef_);
  }

  // 補間をdtすすめる
  primitiveStates.updateTargetForOneStep(dt);
}

void SupportEEFFixer::getPreviewPrimitiveState(const std::string& instance_name, SupportEEFFixer::Ports& port, double dt, primitive_motion_level_tools::PrimitiveStatesSequence& PrimitiveStatesSequence) {
  if(port.m_previewPrimitiveStateRefIn_.isNew()) {
    port.m_previewPrimitiveStateRefIn_.read();

    PrimitiveStatesSequence.updateFromIdl(port.m_previewPrimitiveStateRef_);
  }else{
    // 時刻をdtはやめる
    PrimitiveStatesSequence.updateTargetForOneStep(dt);
  }
}

void SupportEEFFixer::processModeTransition(const std::string& instance_name, SupportEEFFixer::ControlMode& mode, std::shared_ptr<cpp_filters::TwoPointInterpolator<cnoid::Vector3> >& outputCOMOffsetInterpolator, const cnoid::Vector3& prevCOMCom, const primitive_motion_level_tools::PrimitiveStates& primitiveStates){
  switch(mode.now()){
  case SupportEEFFixer::ControlMode::MODE_SYNC_TO_CONTROL:
    mode.setNextMode(SupportEEFFixer::ControlMode::MODE_CONTROL);
    break;
  case SupportEEFFixer::ControlMode::MODE_SYNC_TO_IDLE:
    if(mode.pre() == SupportEEFFixer::ControlMode::MODE_CONTROL){
      std::shared_ptr<primitive_motion_level_tools::PrimitiveState> primitiveState = nullptr;
      for(std::map<std::string, std::shared_ptr<primitive_motion_level_tools::PrimitiveState> >::const_iterator it = primitiveStates.primitiveState().begin(); it != primitiveStates.primitiveState().end() ;it++){
        if(it->second->name() == "com") primitiveState = it->second;
      }
      if(primitiveState){
        outputCOMOffsetInterpolator->reset(prevCOMCom-primitiveState->targetPose().translation(),cnoid::Vector3::Zero(),cnoid::Vector3::Zero());
        outputCOMOffsetInterpolator->setGoal(cnoid::Vector3::Zero(), 3.0);
      }else{
        outputCOMOffsetInterpolator->reset(cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cnoid::Vector3::Zero());
      }
    }
    mode.setNextMode(SupportEEFFixer::ControlMode::MODE_IDLE);
    break;
  }
  mode.update();
}

void SupportEEFFixer::passThrough(const std::string& instance_name, const cnoid::BodyPtr& robot_com, std::shared_ptr<cpp_filters::TwoPointInterpolator<cnoid::Vector3> >& outputCOMOffsetInterpolator, const primitive_motion_level_tools::PrimitiveStates& primitiveStates, double dt, cnoid::Vector3& prevCOMCom){
  std::shared_ptr<primitive_motion_level_tools::PrimitiveState> primitiveState = nullptr;
  for(std::map<std::string, std::shared_ptr<primitive_motion_level_tools::PrimitiveState> >::const_iterator it = primitiveStates.primitiveState().begin(); it != primitiveStates.primitiveState().end() ;it++){
    if(it->second->name() == "com") primitiveState = it->second;
  }
  if(primitiveState){
    cnoid::Vector3 offset = cnoid::Vector3::Zero();
    if(outputCOMOffsetInterpolator->isEmpty()) outputCOMOffsetInterpolator->get(offset, dt);
    prevCOMCom = primitiveState->targetPose().translation() + offset;
  }else{
    if(outputCOMOffsetInterpolator->isEmpty()) outputCOMOffsetInterpolator->reset(cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cnoid::Vector3::Zero());
    prevCOMCom = robot_com->centerOfMass();
  }
}

void SupportEEFFixer::preProcessForControl(const std::string& instance_name) {
}

void SupportEEFFixer::calcOutputPorts(const std::string& instance_name,
                              SupportEEFFixer::Ports& port,
                              const cnoid::Vector3 prevCOMCom,
                              const Eigen::SparseMatrix<double,Eigen::RowMajor>& M,
                              const Eigen::VectorXd& l,
                              const Eigen::VectorXd& u,
                              bool isRunning,
                              std::vector<Eigen::Vector2d>& vertices,
                              std::vector<Eigen::Vector2d>& previewVertices,
                              double dt,
                              const primitive_motion_level_tools::PrimitiveStates& primitiveStates){
  // primitiveState
  port.m_primitiveStateCom_ = port.m_primitiveStateRef_;
  for(int i=0;i<port.m_primitiveStateCom_.data.length();i++){
    if(port.m_primitiveStateCom_.data[i].time != 0.0) port.m_primitiveStateCom_.data[i].time = dt;
    const cnoid::Position& targetPose = primitiveStates.primitiveState().find(std::string(port.m_primitiveStateCom_.data[i].name))->second->targetPose();
    port.m_primitiveStateCom_.data[i].pose.position.x = targetPose.translation()[0];
    port.m_primitiveStateCom_.data[i].pose.position.y = targetPose.translation()[1];
    port.m_primitiveStateCom_.data[i].pose.position.z = targetPose.translation()[2];
    cnoid::Vector3 rpy = cnoid::rpyFromRot(targetPose.linear());
    port.m_primitiveStateCom_.data[i].pose.orientation.r = rpy[0];
    port.m_primitiveStateCom_.data[i].pose.orientation.p = rpy[1];
    port.m_primitiveStateCom_.data[i].pose.orientation.y = rpy[2];
    const cnoid::Vector6& targetWrench = primitiveStates.primitiveState().find(std::string(port.m_primitiveStateCom_.data[i].name))->second->targetWrench();
    for(size_t j=0;j<6;j++) port.m_primitiveStateCom_.data[i].wrench[j] = targetWrench[j];
  }

  // com
  int comIdx = -1;
  for(int i=0;i<port.m_primitiveStateCom_.data.length();i++){
    if(std::string(port.m_primitiveStateCom_.data[i].name) == "com") comIdx = i;
  }
  if(comIdx == -1 && isRunning){
    // generate com task
    comIdx = port.m_primitiveStateCom_.data.length();
    port.m_primitiveStateCom_.data.length(port.m_primitiveStateCom_.data.length()+1);
    port.m_primitiveStateCom_.data[comIdx].name="com";
    port.m_primitiveStateCom_.data[comIdx].parentLinkName="com";
    port.m_primitiveStateCom_.data[comIdx].localPose.position.x=0.0;
    port.m_primitiveStateCom_.data[comIdx].localPose.position.y=0.0;
    port.m_primitiveStateCom_.data[comIdx].localPose.position.z=0.0;
    port.m_primitiveStateCom_.data[comIdx].localPose.orientation.r=0.0;
    port.m_primitiveStateCom_.data[comIdx].localPose.orientation.p=0.0;
    port.m_primitiveStateCom_.data[comIdx].localPose.orientation.y=0.0;
    port.m_primitiveStateCom_.data[comIdx].supportCOM = false;
    port.m_primitiveStateCom_.data[comIdx].isWrenchCGlobal = false;
    for(int i=0;i<6;i++) port.m_primitiveStateCom_.data[comIdx].wrench[i] = 0.0;
    for(int i=0;i<6;i++) port.m_primitiveStateCom_.data[comIdx].M[i] = 0.0;
    for(int i=0;i<6;i++) port.m_primitiveStateCom_.data[comIdx].D[i] = 0.0;
    for(int i=0;i<6;i++) port.m_primitiveStateCom_.data[comIdx].K[i] = 0.0;
    port.m_primitiveStateCom_.data[comIdx].poseFollowGain[0] = 1.0;
    port.m_primitiveStateCom_.data[comIdx].poseFollowGain[1] = 1.0;
    for(int i=2;i<6;i++) port.m_primitiveStateCom_.data[comIdx].poseFollowGain[i] = 0.0;
    for(int i=0;i<6;i++) port.m_primitiveStateCom_.data[comIdx].wrenchFollowGain[i] = 0.0;
  }
  if(comIdx!=-1){
    // position
    port.m_primitiveStateCom_.data[comIdx].time = dt;
    port.m_primitiveStateCom_.data[comIdx].pose.position.x=prevCOMCom[0];
    port.m_primitiveStateCom_.data[comIdx].pose.position.y=prevCOMCom[1];
    port.m_primitiveStateCom_.data[comIdx].pose.position.z=prevCOMCom[2];
    port.m_primitiveStateCom_.data[comIdx].pose.orientation.r=0.0;
    port.m_primitiveStateCom_.data[comIdx].pose.orientation.p=0.0;
    port.m_primitiveStateCom_.data[comIdx].pose.orientation.y=0.0;
    if(port.m_primitiveStateCom_.data[comIdx].poseFollowGain[0] == 0.0) port.m_primitiveStateCom_.data[comIdx].poseFollowGain[0] = 1.0;
    if(port.m_primitiveStateCom_.data[comIdx].poseFollowGain[1] == 0.0) port.m_primitiveStateCom_.data[comIdx].poseFollowGain[1] = 1.0;

    // cfr
    port.m_primitiveStateCom_.data[comIdx].poseC.length(M.rows());
    port.m_primitiveStateCom_.data[comIdx].poseld.length(l.rows());
    port.m_primitiveStateCom_.data[comIdx].poseud.length(u.rows());
    cnoid::MatrixXd Mdense = M;
    for(int i=0;i<M.rows();i++){
      for(int j=0;j<2;j++) port.m_primitiveStateCom_.data[comIdx].poseC[i][j] = Mdense(i,j);
      for(int j=2;j<6;j++) port.m_primitiveStateCom_.data[comIdx].poseC[i][j] = 0.0;
      port.m_primitiveStateCom_.data[comIdx].poseld[i] = std::max(l[i],-1e10); // 大きすぎると後のコンポーネントの処理でオーバーフローする
      port.m_primitiveStateCom_.data[comIdx].poseud[i] = std::min(u[i],1e10);
    }
    port.m_primitiveStateCom_.data[comIdx].isPoseCGlobal = true;
  }

  port.m_primitiveStateComOut_.write();

  // vertices
  port.m_vertices_.tm = port.m_primitiveStateRef_.tm;
  port.m_vertices_.data.length(vertices.size()*2);
  for(int i=0;i<vertices.size();i++){
    for(int j=0;j<2;j++) port.m_vertices_.data[i*2+j] = vertices[i][j];
  }
  port.m_verticesOut_.write();

  // vertices
  port.m_previewVertices_.tm = port.m_primitiveStateRef_.tm;
  port.m_previewVertices_.data.length(previewVertices.size()*2);
  for(int i=0;i<previewVertices.size();i++){
    for(int j=0;j<2;j++) port.m_previewVertices_.data[i*2+j] = previewVertices[i][j];
  }
  port.m_previewVerticesOut_.write();

}

RTC::ReturnCode_t SupportEEFFixer::onExecute(RTC::UniqueId ec_id){

  std::string instance_name = std::string(this->m_profile.instance_name);
  double dt = 1.0 / this->get_context(ec_id)->get_rate();

  // read ports
  SupportEEFFixer::getPrimitiveState(instance_name, this->ports_, dt, this->primitiveStates_);
  SupportEEFFixer::getPreviewPrimitiveState(instance_name, this->ports_, dt, this->previewPrimitiveStates_);
  SupportEEFFixer::getCommandRobot(instance_name, this->ports_, this->robot_com_);

  // mode遷移を実行
  SupportEEFFixer::processModeTransition(instance_name, this->mode_, this->outputCOMOffsetInterpolator_, this->prevCOMCom_, this->primitiveStates_);

  // 重心world XY座標のFR
  Eigen::SparseMatrix<double,Eigen::RowMajor> M(0,2);
  Eigen::VectorXd l;
  Eigen::VectorXd u;
  std::vector<Eigen::Vector2d> vertices, previewVertices;
  if(this->mode_.isRunning()) {
    if(this->mode_.isInitialize()){
      SupportEEFFixer::preProcessForControl(instance_name);
    }

    // this->scfrController_.control(this->primitiveStates_, this->previewPrimitiveStates_, this->robot_com_->mass(), dt, this->regionMargin_, this->prevCOMCom_, M, l, u, vertices, previewVertices, this->debugLevel_);
  }else{
    SupportEEFFixer::passThrough(instance_name, this->robot_com_, this->outputCOMOffsetInterpolator_, this->primitiveStates_, dt, this->prevCOMCom_);
  }

  // write outport
  SupportEEFFixer::calcOutputPorts(instance_name, this->ports_, this->prevCOMCom_, M, l, u, this->mode_.isRunning(), vertices, previewVertices, dt, this->primitiveStates_);

  this->loop_++;
  return RTC::RTC_OK;
}


bool SupportEEFFixer::startControl(){
  if(this->mode_.now() == ControlMode::MODE_IDLE){
    std::cerr << "[" << m_profile.instance_name << "] "<< "startControl" << std::endl;
    this->mode_.setNextMode(ControlMode::MODE_SYNC_TO_CONTROL);
    return true;
  }else{
    std::cerr << "\x1b[31m[" << m_profile.instance_name << "] " << "Invalid context to startControl" << "\x1b[39m" << std::endl;
    return false;
  }
}


bool SupportEEFFixer::stopControl(){
  if(this->mode_.now() == ControlMode::MODE_CONTROL ){
    std::cerr << "[" << m_profile.instance_name << "] "<< "stopControl" << std::endl;
    this->mode_.setNextMode(ControlMode::MODE_SYNC_TO_IDLE);
    return true;
  }else{
    std::cerr << "\x1b[31m[" << m_profile.instance_name << "] " << "Invalid context to stopControl" << "\x1b[39m" << std::endl;
    return false;
  }
}

bool SupportEEFFixer::setParams(const whole_body_master_slave_choreonoid::SupportEEFFixerService::SupportEEFFixerParam& i_param){
  std::cerr << "[" << m_profile.instance_name << "] "<< "setParams" << std::endl;
  this->debugLevel_ = i_param.debugLevel;
  this->regionMargin_ = i_param.regionMargin;
  return true;
}


bool SupportEEFFixer::getParams(whole_body_master_slave_choreonoid::SupportEEFFixerService::SupportEEFFixerParam& i_param){
  std::cerr << "[" << m_profile.instance_name << "] "<< "getParams" << std::endl;
  i_param.debugLevel = this->debugLevel_;
  i_param.regionMargin = this->regionMargin_;
  return true;
}

RTC::ReturnCode_t SupportEEFFixer::onActivated(RTC::UniqueId ec_id){
  std::cerr << "[" << m_profile.instance_name << "] "<< "onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}
RTC::ReturnCode_t SupportEEFFixer::onDeactivated(RTC::UniqueId ec_id){
  std::cerr << "[" << m_profile.instance_name << "] "<< "onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}
RTC::ReturnCode_t SupportEEFFixer::onFinalize(){ return RTC::RTC_OK; }

extern "C"{
    void SupportEEFFixerInit(RTC::Manager* manager) {
        RTC::Properties profile(SupportEEFFixer_spec);
        manager->registerFactory(profile, RTC::Create<SupportEEFFixer>, RTC::Delete<SupportEEFFixer>);
    }
};
