#include "TorqueController.h"
#include <cnoid/EigenUtil>
#include <cnoid/src/Body/InverseDynamics.h>
#include <cnoid/ValueTree>

namespace WholeBodyTorque {
  TorqueController::PrimitiveTask::PrimitiveTask(const std::string& name) :
    name_(name),
    prevError_(cnoid::Vector6::Zero())
  {
  }

  void TorqueController::PrimitiveTask::calcInteractTorque(const cnoid::BodyPtr& robot_act, cnoid::Vector6& rootWrench, double dt, const std::vector<cnoid::LinkPtr>& useJoints, int debugLevel){
    if(this->name_ == "com" || this->primitiveCommand_->supportCOM()) return;

    if(!robot_act->link(this->primitiveCommand_->parentLinkName())) {
      std::cerr << "\x1b[31m[TorqueController::PrimitiveTask::calcInteractTorque] [" << this->primitiveCommand_->parentLinkName() <<"] doesn't exist" << "\x1b[39m" << std::endl;
      return;
    }

    PrimitiveTask::calcPositionConstraint(robot_act, this->primitiveCommand_, this->positionConstraint_);
    this->positionConstraint_->debuglevel() = debugLevel;
    this->positionConstraint_->checkConvergence();
    cnoid::Vector6 error = this->positionConstraint_->calc_error(); // actual - reference. world系
    std::vector<cnoid::LinkPtr> useJointsAct; for(int i=0;i<useJoints.size();i++) useJointsAct.push_back(robot_act->link(useJoints[i]->name()));
    const Eigen::SparseMatrix<double,Eigen::RowMajor>& jacobian =  this->positionConstraint_->calc_jacobian(useJointsAct); // actual - reference. world系, endeffectorまわり

    cnoid::Vector6 derror = (error - this->prevError_) / dt;

    cnoid::Vector6 errorLocal, derrorLocal;
    errorLocal.head<3>() = this->primitiveCommand_->targetPose().linear().transpose() * error.head<3>();
    errorLocal.tail<3>() = this->primitiveCommand_->targetPose().linear().transpose() * error.tail<3>();
    derrorLocal.head<3>() = this->primitiveCommand_->targetPose().linear().transpose() * derror.head<3>();
    derrorLocal.tail<3>() = this->primitiveCommand_->targetPose().linear().transpose() * derror.tail<3>();

    cnoid::Vector6 targetWrenchLocal = cnoid::Vector6::Zero(); // robot receive
    targetWrenchLocal += (errorLocal.cwiseProduct(this->primitiveCommand_->K()) + errorLocal.cwiseProduct(this->primitiveCommand_->D()) ).cwiseProduct(this->primitiveCommand_->poseFollowGain());
    {
      cnoid::Vector6 refWrench = this->primitiveCommand_->targetWrench();
      cnoid::Vector6 refWrenchLocal;
      refWrenchLocal.head<3>() = this->primitiveCommand_->targetPose().linear().transpose() * refWrench.head<3>();
      refWrenchLocal.tail<3>() = this->primitiveCommand_->targetPose().linear().transpose() * refWrench.tail<3>();
      targetWrenchLocal += refWrenchLocal.cwiseProduct(this->primitiveCommand_->wrenchFollowGain());
    }
    cnoid::Vector6 targetWrench; // robot receive
    targetWrench.head<3>() = this->primitiveCommand_->targetPose().linear() * targetWrenchLocal.head<3>();
    targetWrench.tail<3>() = this->primitiveCommand_->targetPose().linear() * targetWrenchLocal.tail<3>();

    cnoid::VectorX torque = - jacobian.transpose() * targetWrench;

    for(int i=0, idx=0;i<useJoints.size();i++){
      int dof = IK::IKConstraint::getJointDOF(useJoints[i]);
      if(useJoints[i]->isRoot() && !useJoints[i]->isFixedJoint()) rootWrench += torque.segment<6>(idx);
      else useJoints[i]->u() += torque[idx];
      idx += IK::IKConstraint::getJointDOF(useJoints[i]);
    }

    this->prevError_ = error;
  }

  void TorqueController::PrimitiveTask::calcPositionConstraint(const cnoid::BodyPtr& robot,
                                                               const std::shared_ptr<const primitive_motion_level_tools::PrimitiveState>& primitiveCommand,
                                                               std::shared_ptr<IK::PositionConstraint>& positionConstraint){
    if(!robot->link(primitiveCommand->parentLinkName())){
      std::cerr << "link " << primitiveCommand->parentLinkName() << " not exist" << std::endl;
      positionConstraint = nullptr;
      return;
    }
    if(!positionConstraint) positionConstraint = std::make_shared<IK::PositionConstraint>();
    positionConstraint->A_link() = robot->link(primitiveCommand->parentLinkName());
    positionConstraint->A_localpos() = primitiveCommand->localPose();
    positionConstraint->B_link() = nullptr;
    positionConstraint->B_localpos() = primitiveCommand->targetPose();
    positionConstraint->maxError() << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    positionConstraint->weight() << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    positionConstraint->eval_link() = nullptr;
    positionConstraint->eval_localR() = positionConstraint->B_localpos().linear();
  }

  void TorqueController::reset() {
    this->positionTaskMap_.clear();
  }

  void TorqueController::calcGravityCompensation(cnoid::BodyPtr& robot_act, cnoid::Vector6& rootWrench, const std::vector<cnoid::LinkPtr>& useJoints) {
    cnoid::Vector3 vorg = robot_act->rootLink()->v();
    cnoid::Vector3 worg = robot_act->rootLink()->w();
    cnoid::Vector3 dvorg = robot_act->rootLink()->dv();
    cnoid::Vector3 dworg = robot_act->rootLink()->dw();
    robot_act->rootLink()->v() << 0.0, 0.0, 0.0;
    robot_act->rootLink()->w() << 0.0, 0.0, 0.0;
    robot_act->rootLink()->dv() << 0.0, 0.0, 9.80665;
    robot_act->rootLink()->dw() << 0.0, 0.0, 0.0;
    cnoid::VectorX dqorg(robot_act->numAllJoints());
    cnoid::VectorX ddqorg(robot_act->numAllJoints());
    for(size_t i=0;i<robot_act->numAllJoints();i++){
      dqorg[i]=robot_act->joint(i)->dq();
      robot_act->joint(i)->dq() = 0.0;
      ddqorg[i]=robot_act->joint(i)->ddq();
      robot_act->joint(i)->ddq() = 0.0;
    }
    robot_act->calcForwardKinematics(true,true);
    cnoid::Vector6 rootGravityCompensationWrench = cnoid::calcInverseDynamics(robot_act->rootLink());

    for(int i=0;i<useJoints.size();i++) {
      if(useJoints[i]->isRoot() && !useJoints[i]->isFixedJoint()) rootWrench += rootGravityCompensationWrench;
      else useJoints[i]->u() += robot_act->joint(useJoints[i]->jointId())->u();
    }

    robot_act->rootLink()->v() = vorg;
    robot_act->rootLink()->w() = worg;
    robot_act->rootLink()->dv() = dvorg;
    robot_act->rootLink()->dw() = dworg;
    for(size_t i=0;i<robot_act->numAllJoints();i++){
      robot_act->joint(i)->dq() = dqorg[i];
      robot_act->joint(i)->ddq() = ddqorg[i];
    }
  }

  void TorqueController::getPrimitiveCommand(const std::map<std::string, std::shared_ptr<primitive_motion_level_tools::PrimitiveState> >& primitiveCommandMap, std::map<std::string, std::shared_ptr<TorqueController::PrimitiveTask> >& positionTaskMap) {
    // 消滅したEndEffectorを削除
    for(std::map<std::string, std::shared_ptr<TorqueController::PrimitiveTask> >::iterator it = positionTaskMap.begin(); it != positionTaskMap.end(); ) {
      if (primitiveCommandMap.find(it->first) == primitiveCommandMap.end()) it = positionTaskMap.erase(it);
      else ++it;
    }
    // 増加したEndEffectorの反映
    for(std::map<std::string, std::shared_ptr<primitive_motion_level_tools::PrimitiveState> >::const_iterator it = primitiveCommandMap.begin(); it != primitiveCommandMap.end(); it++) {
      if(positionTaskMap.find(it->first)==positionTaskMap.end()){
        positionTaskMap[it->first] = std::make_shared<TorqueController::PrimitiveTask>(it->first);
      }
    }
    // 各指令値の反映
    for(std::map<std::string, std::shared_ptr<primitive_motion_level_tools::PrimitiveState> >::const_iterator it = primitiveCommandMap.begin(); it != primitiveCommandMap.end(); it++){
      positionTaskMap[it->first]->updateFromPrimitiveCommand(it->second);
    }
  }

  void TorqueController::calcJointLimitTorque(cnoid::BodyPtr& robot_act, const std::vector<cnoid::LinkPtr>& useJoints) {
    for(int i=0;i<useJoints.size();i++){
      if(useJoints[i]->jointId()>=0){
        const double soft_ulimit = useJoints[i]->q_upper() - 0.261799; // 15 deg
        const double soft_llimit = useJoints[i]->q_lower() + 0.261799;
        double soft_jlimit_tq = 0;
        double qAct = robot_act->joint(useJoints[i]->jointId())->q();
        if(qAct > soft_ulimit){ soft_jlimit_tq = 100 * (soft_ulimit - qAct); }
        if(qAct < soft_llimit){ soft_jlimit_tq = 100 * (soft_llimit - qAct); }
        useJoints[i]->u() += soft_jlimit_tq;
      }
    }
  }

  void TorqueController::control(const std::map<std::string, std::shared_ptr<primitive_motion_level_tools::PrimitiveState> >& primitiveCommandMap, // primitive motion level target
                                 const std::vector<std::shared_ptr<WholeBodyTorque::Collision> >& collisions, // current self collision state
                                 const cnoid::BodyPtr& robot_ref, // command level target
                                 cnoid::BodyPtr& robot_act,
                                 const std::vector<cnoid::LinkPtr>& useJoints, // input and output
                                 std::unordered_map<cnoid::LinkPtr, std::vector<std::shared_ptr<joint_limit_table::JointLimitTable> > >& jointLimitTablesMap,
                                 double dt,
                                 int debugLevel
                                 ) {
    for(int i=0;i<useJoints.size();i++) useJoints[i]->u() = 0.0;
    cnoid::Vector6 rootWrench = cnoid::Vector6::Zero();

    // 重力補償トルクを足す
    TorqueController::calcGravityCompensation(robot_act, rootWrench, useJoints);
    for(int i=0;i<useJoints.size();i++) std::cerr << useJoints[i]->u() <<std::endl;

    // command level指令の関節角度に追従するトルクを足す
    //  モータドライバで行う TODO
    for(size_t i=0;i<useJoints.size();i++) if(useJoints[i]->jointId()>=0) useJoints[i]->q() = robot_ref->joint(useJoints[i]->jointId())->q();
    for(size_t i=0;i<useJoints.size();i++) if(useJoints[i]->jointId()>=0) useJoints[i]->u() += - 10 * robot_act->joint(useJoints[i]->jointId())->dq();

    // soft joint limit トルクを足す
    TorqueController::calcJointLimitTorque(robot_act, useJoints);

    for(int i=0;i<useJoints.size();i++) std::cerr << useJoints[i]->u() <<std::endl;

    // 目標primitive commandを取得
    TorqueController::getPrimitiveCommand(primitiveCommandMap, this->positionTaskMap_);

    // 各primitive command実現のためのトルクを足す (supportCOM, comを除く)
    for(std::map<std::string, std::shared_ptr<TorqueController::PrimitiveTask> >::iterator it = this->positionTaskMap_.begin(); it != this->positionTaskMap_.end(); it++) {
      it->second->calcInteractTorque(robot_act, rootWrench, dt, useJoints, debugLevel);
    }

    // 各primitive command実現のためのトルクを足す (supportCOM, com)
    // TODO

    // トルク上限でリミット
    for(int i=0;i<useJoints.size();i++){
      double climit, gearRatio, torqueConst;
      useJoints[i]->info()->read("climit",climit);
      useJoints[i]->info()->read("gearRatio",gearRatio);
      useJoints[i]->info()->read("torqueConst",torqueConst);
      double maxTorque = climit * gearRatio * torqueConst;
      useJoints[i]->u() = std::min(std::max(useJoints[i]->u(), -maxTorque), maxTorque);
    }

    for(int i=0;i<useJoints.size();i++) std::cerr << useJoints[i]->u() <<std::endl;

  }



}
