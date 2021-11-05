#include "InternalWrenchController.h"
#include <cnoid/ValueTree>

namespace whole_body_master_slave_choreonoid {
  void InternalWrenchController::control(const primitive_motion_level_tools::PrimitiveStates& primitiveStates,
                                         std::unordered_map<std::string, std::shared_ptr<cpp_filters::FirstOrderLowPassFilter<cnoid::Vector6> > >& wrenchFilterMap,
                                         const cnoid::BodyPtr& robot_act,
                                         const std::vector<double>& pgain,
                                         const std::vector<cnoid::LinkPtr>& useJoints,
                                         double dt,
                                         int debugLevel,
                                         std::unordered_map<std::string, cnoid::Position>& fixedPoseMap //input & output
                                         ) {
    if(this->startFlag_) {
      // 修正量を計算
      this->startFlag_ = false;
      if(this->calcEEModification(primitiveStates, wrenchFilterMap, robot_act, pgain, useJoints, fixedPoseMap, debugLevel)) this->remainTime_ = this->transitionTime_;
    }

    // 補間が終了
    if(this->remainTime_ == 0.0) {
      transitionVelocityMap_.clear();
      return;
    }

    // calcEEModification時からsupportEEFが変わっていたら止まる
    bool changed = false;
    for(std::unordered_map<std::string, cnoid::Position>::iterator it=fixedPoseMap.begin(); it != fixedPoseMap.end(); it++) {
      if(this->transitionVelocityMap_.find(it->first) == this->transitionVelocityMap_.end()) changed = true;
    }
    for(std::unordered_map<std::string, cnoid::Vector6>::iterator it=this->transitionVelocityMap_.begin(); it != this->transitionVelocityMap_.end(); it++) {
      if(fixedPoseMap.find(it->first) == fixedPoseMap.end()) changed = true;
    }
    if(changed) {
      this->remainTime_ = 0.0;
      transitionVelocityMap_.clear();
      return;
    }

    // fixedPoseMapを相対的に動かす
    for(std::unordered_map<std::string, cnoid::Position>::iterator it=fixedPoseMap.begin(); it != fixedPoseMap.end(); it++) {
      cnoid::Vector6 delta = this->transitionVelocityMap_[it->first] * dt; //local
      it->second.translation() += it->second.linear() * delta.head<3>();
      if(delta.tail<3>().norm() != 0.0)
        it->second.linear() = Eigen::Matrix3d(Eigen::AngleAxisd(it->second.linear()) * Eigen::AngleAxisd(delta.tail<3>().norm(),delta.tail<3>().normalized()));
    }
    this->remainTime_ = std::max(0.0, this->remainTime_ - dt);

    return;
  }

  bool InternalWrenchController::calcEEModification(const primitive_motion_level_tools::PrimitiveStates& primitiveStates,
                                                    std::unordered_map<std::string, std::shared_ptr<cpp_filters::FirstOrderLowPassFilter<cnoid::Vector6> > >& wrenchFilterMap,
                                                    const cnoid::BodyPtr& robot_act,
                                                    const std::vector<double>& pgain,
                                                    const std::vector<cnoid::LinkPtr>& useJoints,
                                                    const std::unordered_map<std::string, cnoid::Position>& fixedPoseMap,
                                                    int debugLevel) {
    std::vector<cnoid::LinkPtr> useJointsReal;
    std::vector<double> pgainReal;
    int numJoints = 0;
    useJointsReal.push_back(robot_act->rootLink());
    for(int i=0;i<useJoints.size();i++){
      if(pgain[useJoints[i]->jointId()] > 0.0) {
        useJointsReal.push_back(useJoints[i]);
        pgainReal.push_back(pgain[useJoints[i]->jointId()]);
        numJoints++;
      }
    }

    // 消滅したsupportEEFを削除
    for(std::unordered_map<std::string, std::shared_ptr<IK::PositionConstraint> >::iterator it = this->positionConstraintMap_.begin(); it != this->positionConstraintMap_.end(); ){
      if(fixedPoseMap.find(it->first) == fixedPoseMap.end()) it = this->positionConstraintMap_.erase(it);
      else ++it;
    }

    // wrenchはlocal系. delta
    std::vector<Eigen::SparseMatrix<double, Eigen::ColMajor> > Jts; // 6 x 6+n
    std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> > wrenchAs; // ? x 6
    std::vector<cnoid::VectorX> wrenchbs; // ?
    std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> > wrenchCs; // ? x 6
    std::vector<cnoid::VectorX> wrenchdus; // ?
    std::vector<cnoid::VectorX> wrenchdls; // ?
    for(std::unordered_map<std::string, cnoid::Position>::const_iterator it=fixedPoseMap.begin(); it != fixedPoseMap.end(); it++) {
      std::shared_ptr<primitive_motion_level_tools::PrimitiveState> primitiveState = primitiveStates.primitiveState().find(it->first)->second;
      cnoid::Position pose = robot_act->link(primitiveState->parentLinkName())->T() * primitiveState->localPose();

      if(this->positionConstraintMap_.find(it->first) == this->positionConstraintMap_.end()){
        this->positionConstraintMap_[it->first] = std::make_shared<IK::PositionConstraint>();
      }
      std::shared_ptr<IK::PositionConstraint>& positionConstraint = this->positionConstraintMap_[it->first];
      positionConstraint->A_link() = robot_act->link(primitiveState->parentLinkName());
      positionConstraint->A_localpos() = primitiveState->localPose();
      positionConstraint->eval_link() = robot_act->link(primitiveState->parentLinkName());
      positionConstraint->eval_localR() = primitiveState->localPose().linear();
      positionConstraint->calc_error();
      Jts.push_back(positionConstraint->calc_jacobian(useJointsReal).transpose()); // endeffector系, endeffectorまわり. 6 x root + joints

      Eigen::SparseMatrix<double, Eigen::RowMajor> Rt(6,6);
      {
        cnoid::Matrix3 Rt_ = primitiveState->targetPose().linear().transpose();
        for(int i=0;i<3;i++) {
          for(int j=0;j<3;j++) {
            Rt.insert(i,j) = Rt_(i,j);
            Rt.insert(3+i,3+j) = Rt_(i,j);
          }
        }
      }

      Eigen::SparseMatrix<double, Eigen::RowMajor> A(6,6);
      cnoid::Vector6 b;
      cnoid::Vector6 wrenchRef = Rt * primitiveState->targetWrench();
      for(int i=0;i<6;i++){
        A.insert(i,i) = primitiveState->wrenchFollowGain()[i];
        b(i) = wrenchRef[i] * primitiveState->wrenchFollowGain()[i];
      }
      wrenchAs.push_back(A);
      wrenchbs.push_back(b - A * wrenchFilterMap[it->first]->getCurrentValue());

      Eigen::SparseMatrix<double, Eigen::RowMajor> C;
      if(primitiveState->isWrenchCGlobal()) {
        C = primitiveState->wrenchC() * Rt.transpose();
      }else{
        C = primitiveState->wrenchC();
      }
      wrenchCs.push_back(C);
      wrenchdus.push_back(primitiveState->wrenchud() - C * wrenchFilterMap[it->first]->getCurrentValue());
      wrenchdls.push_back(primitiveState->wrenchld() - C * wrenchFilterMap[it->first]->getCurrentValue());
    }

    Eigen::SparseMatrix<double, Eigen::ColMajor> Jt;
    InternalWrenchController::appendCol(Jts, Jt);
    Eigen::SparseMatrix<double, Eigen::RowMajor> wrenchA;
    InternalWrenchController::appendDiag(wrenchAs, wrenchA);
    cnoid::VectorX wrenchb;
    InternalWrenchController::appendRow(wrenchbs, wrenchb);
    Eigen::SparseMatrix<double, Eigen::RowMajor> wrenchC;
    InternalWrenchController::appendDiag(wrenchCs, wrenchC);
    cnoid::VectorX wrenchdl;
    InternalWrenchController::appendRow(wrenchdls, wrenchdl);
    cnoid::VectorX wrenchdu;
    InternalWrenchController::appendRow(wrenchdus, wrenchdu);

    Eigen::SparseMatrix<double, Eigen::RowMajor> Jt_RowMajor = Jt;
    Eigen::SparseMatrix<double, Eigen::RowMajor> Jt_root = Jt_RowMajor.topRows<6>();
    Eigen::SparseMatrix<double, Eigen::RowMajor> Jt_joints = Jt_RowMajor.bottomRows(Jt_RowMajor.rows()-6);

    Eigen::SparseMatrix<double, Eigen::RowMajor> tauA(numJoints,numJoints);
    cnoid::VectorX taub(numJoints);
    Eigen::SparseMatrix<double, Eigen::RowMajor> tauC(numJoints,numJoints);
    cnoid::VectorX taudl(numJoints);
    cnoid::VectorX taudu(numJoints);
    for(int i=0;i<numJoints;i++){
      cnoid::LinkPtr joint = useJointsReal[1+i];
      double climit, gearRatio, torqueConst;
      joint->info()->read("climit",climit);
      joint->info()->read("gearRatio",gearRatio);
      joint->info()->read("torqueConst",torqueConst);
      double maxTorque = std::max(climit * gearRatio * torqueConst, 0.01);
      tauA.insert(i,i) = 1.0 / maxTorque;
      taub[i] = 0.0 - joint->u() / maxTorque;
      tauC.insert(i,i) = 1.0;
      taudl[i] = - maxTorque - joint->u();
      taudu[i] = maxTorque - joint->u();
    }

    int dim = Jt.cols();
    std::vector<std::shared_ptr<prioritized_qp_base::Task> > tasks;
    {
      if(!this->rootBalanceTask_){
        this->rootBalanceTask_ = std::make_shared<prioritized_qp::Task>();
        this->rootBalanceTask_->name() = "rootBalanceTask";
      }
      this->rootBalanceTask_->A() = Jt_root;
      this->rootBalanceTask_->b() = cnoid::Vector6::Zero();
      this->rootBalanceTask_->C().resize(0,dim);
      this->rootBalanceTask_->wa() = cnoid::Vector6::Ones();
      this->rootBalanceTask_->w() = cnoid::VectorX::Ones(dim) * 1e-6;
      this->rootBalanceTask_->toSolve() = false;
      this->rootBalanceTask_->solver().settings()->setVerbosity(debugLevel);
      tasks.push_back(this->rootBalanceTask_);
    }
    {
      if(!this->tauLimitTask_){
        this->tauLimitTask_ = std::make_shared<prioritized_qp::Task>();
        this->tauLimitTask_->name() = "tauLimitTask";
      }
      this->tauLimitTask_->A().resize(0,dim);
      this->tauLimitTask_->C() = tauC * - Jt_joints;
      this->tauLimitTask_->dl() = taudl;
      this->tauLimitTask_->du() = taudu;
      this->tauLimitTask_->wc() = cnoid::VectorX::Ones(taudu.size());
      this->tauLimitTask_->w() = cnoid::VectorX::Ones(dim) * 1e-6;
      this->tauLimitTask_->toSolve() = true;
      this->tauLimitTask_->solver().settings()->setVerbosity(debugLevel);
      tasks.push_back(this->tauLimitTask_);
    }
    {
      if(!this->wrenchLimitTask_){
        this->wrenchLimitTask_ = std::make_shared<prioritized_qp::Task>();
        this->wrenchLimitTask_->name() = "wrenchLimitTask";
      }
      this->wrenchLimitTask_->A().resize(0,dim);
      this->wrenchLimitTask_->C() = wrenchC;
      this->wrenchLimitTask_->dl() = wrenchdl;
      this->wrenchLimitTask_->du() = wrenchdu;
      this->wrenchLimitTask_->wc() = cnoid::VectorX::Ones(wrenchdu.size());
      this->wrenchLimitTask_->w() = cnoid::VectorX::Ones(dim) * 1e-6;
      this->wrenchLimitTask_->toSolve() = true;
      this->wrenchLimitTask_->solver().settings()->setVerbosity(debugLevel);
      tasks.push_back(this->wrenchLimitTask_);
    }
    {
      if(!this->wrenchTargetTask_){
        this->wrenchTargetTask_ = std::make_shared<prioritized_qp::Task>();
        this->wrenchTargetTask_->name() = "wrenchTargetTask";
      }
      this->wrenchTargetTask_->A() = wrenchA;
      this->wrenchTargetTask_->b() = wrenchb;
      this->wrenchTargetTask_->C().resize(0,dim);
      this->wrenchTargetTask_->wa() = cnoid::VectorX::Ones(wrenchb.size());
      this->wrenchTargetTask_->w() = cnoid::VectorX::Ones(dim) * 1e-6;
      this->wrenchTargetTask_->toSolve() = true;
      this->wrenchTargetTask_->solver().settings()->setVerbosity(debugLevel);
      tasks.push_back(this->wrenchTargetTask_);
    }
    {
      if(!this->taumaxDefineTask_){
        this->taumaxDefineTask_ = std::make_shared<prioritized_qp::Task>();
        this->taumaxDefineTask_->name() = "Define taumax";
      }
      this->taumaxDefineTask_->id_ext().resize(1);
      this->taumaxDefineTask_->id_ext()[0] = "taumax";
      this->taumaxDefineTask_->A().resize(0,dim);
      this->taumaxDefineTask_->A_ext().resize(0,1);
      this->taumaxDefineTask_->C().resize(tauA.rows()*2,dim);
      this->taumaxDefineTask_->C().topRows(tauA.rows()) = tauA * - Jt_joints;
      this->taumaxDefineTask_->C().bottomRows(tauA.rows()) = tauA * - Jt_joints;
      this->taumaxDefineTask_->C_ext().resize(tauA.rows()*2,1);
      for(int i=0;i<tauA.rows();i++) this->taumaxDefineTask_->C_ext().coeffRef(i,0) = 1.0;
      for(int i=tauA.rows();i<tauA.rows()*2;i++) this->taumaxDefineTask_->C_ext().coeffRef(i,0) = -1.0;
      this->taumaxDefineTask_->dl().resize(taub.rows()*2);
      this->taumaxDefineTask_->dl().head(taub.rows()) = taub;
      this->taumaxDefineTask_->dl().tail(taub.rows()) = cnoid::VectorX::Ones(taub.rows()) * - 1e10;
      this->taumaxDefineTask_->du().resize(taub.rows()*2);
      this->taumaxDefineTask_->du().head(taub.rows()) = cnoid::VectorX::Ones(taub.rows()) * 1e10;
      this->taumaxDefineTask_->du().tail(taub.rows()) = taub;
      this->taumaxDefineTask_->wc() = cnoid::VectorX::Ones(taub.size()*2);
      this->taumaxDefineTask_->w() = cnoid::VectorX::Ones(dim) * 1e-6;
      this->taumaxDefineTask_->w_ext() = cnoid::VectorX::Ones(1) * 1e-6;
      this->taumaxDefineTask_->toSolve() = false;
      this->taumaxDefineTask_->solver().settings()->setVerbosity(debugLevel);
      tasks.push_back(this->taumaxDefineTask_);
    }
    {
      if(!this->tauTargetTask_){
        this->tauTargetTask_ = std::make_shared<prioritized_qp::Task>();
        this->tauTargetTask_->name() = "tauTargetTask";
      }
      this->tauTargetTask_->id_ext().resize(1);
      this->tauTargetTask_->id_ext()[0] = "taumax";
      this->tauTargetTask_->A() = Eigen::SparseMatrix<double,Eigen::RowMajor>(tauA.rows()+1,dim);
      this->tauTargetTask_->A().topRows(tauA.rows()) = tauA * - Jt_joints;
      this->tauTargetTask_->A_ext() = Eigen::SparseMatrix<double,Eigen::RowMajor>(tauA.rows()+1,1);
      this->tauTargetTask_->A_ext().insert(tauA.rows(),0) = 1.0;
      this->tauTargetTask_->b().resize(tauA.rows()+1);
      this->tauTargetTask_->b().head(tauA.rows()) = taub;
      this->tauTargetTask_->b()(tauA.rows()) = 0.0;
      this->tauTargetTask_->C().resize(0,dim);
      this->tauTargetTask_->C_ext().resize(0,1);
      this->tauTargetTask_->wa() = cnoid::VectorX::Ones(taub.size()+1);
      this->tauTargetTask_->wa()[taub.size()] = 10.0;
      this->tauTargetTask_->w() = cnoid::VectorX::Ones(dim) * 1e-6;
      this->tauTargetTask_->w_ext() = cnoid::VectorX::Ones(1) * 1e-6;
      this->tauTargetTask_->toSolve() = true;
      this->tauTargetTask_->solver().settings()->setVerbosity(debugLevel);
      tasks.push_back(this->tauTargetTask_);
    }

    cnoid::VectorX result;
    if(!prioritized_qp_base::solve(tasks, result, debugLevel)){
      return false;
    }

    Eigen::SparseMatrix<double, Eigen::RowMajor> Kinv(pgainReal.size(),pgainReal.size());
    for(int i=0;i<pgainReal.size();i++) Kinv.insert(i,i) = 1.0 / pgainReal[i];
    cnoid::VectorX dp = Jt_joints.transpose() * Kinv * - Jt_joints * result;

    this->transitionVelocityMap_.clear();
    if( this->transitionTime_ <= 0.0) this->transitionTime_ = 1.0;
    int i=0;
    for(std::unordered_map<std::string, cnoid::Position>::const_iterator it=fixedPoseMap.begin(); it != fixedPoseMap.end(); it++) {
      transitionVelocityMap_[it->first] = dp.segment<6>(6*i) / this->transitionTime_;
      i++;
    }

    return true;
  }


  bool InternalWrenchController::appendRow(const std::vector<cnoid::VectorX>& vs, cnoid::VectorX& vout){
    size_t rows = 0;
    for(size_t i=0;i<vs.size();i++){
      rows += vs[i].size();
    }
    vout.resize(rows);
    size_t idx = 0;
    for(size_t i=0;i<vs.size();i++){
      vout.segment(idx,vs[i].size()) = vs[i];
      idx += vs[i].size();
    }

    return true;
  }

  bool InternalWrenchController::appendCol(const std::vector<Eigen::SparseMatrix<double, Eigen::ColMajor> >& Ms, Eigen::SparseMatrix<double, Eigen::ColMajor >& Mout){
    if(Ms.size() == 0) {
      Mout.resize(Mout.rows(),0);//もとのMoutのrowを用いる
      return true;
    }
    size_t rows = Ms[0].rows();
    size_t cols = 0;
    for(size_t i=0;i<Ms.size();i++){
      cols += Ms[i].cols();
      if(Ms[i].rows() != rows){
        std::cerr << "[appendCol] Ms[i].rows() " << Ms[i].rows() << " != rows " << rows << std::endl;
        return false;
      }
    }
    Mout.resize(rows,cols);
    size_t idx = 0;
    for(size_t i=0;i<Ms.size();i++){
      Mout.middleCols(idx,Ms[i].cols()) = Ms[i];
      idx += Ms[i].cols();
    }

    return true;
  }

  bool InternalWrenchController::appendDiag(const std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> >& Ms, Eigen::SparseMatrix<double, Eigen::RowMajor >& Mout){
    if(Ms.size() == 0) {
      Mout.resize(0,0);
      return true;
    }
    size_t cols = 0;
    size_t rows = 0;
    for(size_t i=0;i<Ms.size();i++){
      rows += Ms[i].rows();
      cols += Ms[i].cols();
    }
    Mout.resize(rows,cols);
    size_t idx_row = 0;
    size_t idx_col = 0;
    for(size_t i=0;i<Ms.size();i++){
      Eigen::SparseMatrix<double, Eigen::ColMajor> M_ColMajor(Ms[i].rows(),cols);
      M_ColMajor.middleCols(idx_col,Ms[i].cols()) = Ms[i];
      Mout.middleRows(idx_row,M_ColMajor.rows()) = M_ColMajor;
      idx_row += Ms[i].rows();
      idx_col += Ms[i].cols();
    }

    return true;
  }

}
