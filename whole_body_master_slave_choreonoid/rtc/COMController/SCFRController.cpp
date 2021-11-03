#include "SCFRController.h"

namespace whole_body_master_slave_choreonoid{
  void SCFRController::control(const primitive_motion_level_tools::PrimitiveStates& primitiveStates,
                               const primitive_motion_level_tools::PrimitiveStatesSequence& previewPrimitiveStates,
                               double m,
                               double dt,
                               double regionMargin,
                               cnoid::Vector3& prevCOMCom,
                               Eigen::SparseMatrix<double,Eigen::RowMajor>& M,
                               cnoid::VectorX& l,
                               cnoid::VectorX& u,
                               std::vector<Eigen::Vector2d>& vertices,
                               std::vector<Eigen::Vector2d>& previewVertices,
                               int debugLevel){

    const double h = 1.0;
    const double g = 9.80665;

    SCFRController::calcCurrentCFR(primitiveStates,m,this->currentCFRCalculator_,M,l,u,vertices,debugLevel);

    Eigen::SparseMatrix<double,Eigen::RowMajor> Mnext;
    cnoid::VectorX lnext;
    cnoid::VectorX unext;
    SCFRController::calcNextCFR(previewPrimitiveStates,m,this->nextCFRCalculator_,Mnext,lnext,unext,previewVertices,debugLevel);

    double transitionTime = dt;
    if(previewPrimitiveStates.primitiveStates().size() > 0 && previewPrimitiveStates.primitiveStates()[0]->time() > dt) transitionTime = previewPrimitiveStates.primitiveStates()[0]->time();

    /*
      https://github.com/ishiguroJSK/hrpsys-base/blob/22c1708f9deff3244e094ca079c46ff3babf0f99/rtc/WholeBodyMasterSlave/wbms_core.h#L729 の方法
      下記優先順位
      速度のCurrentCFRの上下限をregionMarginで満たす,
      transitionTime後に速度のNextCFR上下限をregionMarginで満たす
      COMRefがあれば追従する or 無ければ安定余裕最大化
    */

    std::vector<std::shared_ptr<prioritized_qp_base::Task> > tasks;

    {
      cnoid::VectorX DCM_l = l - M * prevCOMCom.head<2>() + cnoid::VectorX::Ones(l.rows()) * regionMargin;
      cnoid::VectorX DCM_u = u - M * prevCOMCom.head<2>() - cnoid::VectorX::Ones(u.rows()) * regionMargin;
      Eigen::SparseMatrix<double,Eigen::RowMajor> DCM_M = M * sqrt(h/g) / dt;

      cnoid::VectorX CCM_l = M * prevCOMCom.head<2>() - u + cnoid::VectorX::Ones(u.rows()) * regionMargin;
      for(int i=0;i<CCM_l.size();i++) CCM_l[i] = std::min(CCM_l[i], -0.005);
      cnoid::VectorX CCM_u = M * prevCOMCom.head<2>() - l - cnoid::VectorX::Ones(l.rows()) * regionMargin;
      for(int i=0;i<CCM_u.size();i++) CCM_u[i] = std::max(CCM_u[i], 0.005);
      Eigen::SparseMatrix<double,Eigen::RowMajor> CCM_M = M * sqrt(h/g) / dt;

      if(!this->currentCFRTask_) {
        this->currentCFRTask_ = std::make_shared<prioritized_qp::Task>();
        this->currentCFRTask_->name() = "currentCFRTask";
      }

      this->currentCFRTask_->A().resize(0,2);
      this->currentCFRTask_->C().resize(DCM_M.rows()+CCM_M.rows(),2);
      this->currentCFRTask_->C().topRows(DCM_M.rows()) = DCM_M;
      this->currentCFRTask_->C().bottomRows(CCM_M.rows()) = CCM_M;
      this->currentCFRTask_->dl().resize(DCM_l.rows()+CCM_l.rows());
      this->currentCFRTask_->dl().head(DCM_l.rows()) = DCM_l;
      this->currentCFRTask_->dl().tail(CCM_l.rows()) = CCM_l;
      this->currentCFRTask_->du().resize(DCM_u.rows()+CCM_u.rows());
      this->currentCFRTask_->du().head(DCM_u.rows()) = DCM_u;
      this->currentCFRTask_->du().tail(CCM_u.rows()) = CCM_u;
      this->currentCFRTask_->wc() = cnoid::VectorX::Ones(DCM_l.rows()+CCM_l.rows());
      this->currentCFRTask_->w() = cnoid::VectorX::Ones(2) * 1e-6;
      this->currentCFRTask_->toSolve() = true;
      this->currentCFRTask_->solver().settings()->setVerbosity(debugLevel);

      tasks.push_back(this->currentCFRTask_);
    }

    if(Mnext.rows()>0) {
      cnoid::VectorX DCM_l = lnext - Mnext * prevCOMCom.head<2>() + cnoid::VectorX::Ones(lnext.rows()) * regionMargin;
      cnoid::VectorX DCM_u = unext - Mnext * prevCOMCom.head<2>() - cnoid::VectorX::Ones(unext.rows()) * regionMargin;
      Eigen::SparseMatrix<double,Eigen::RowMajor> DCM_M = Mnext * (sqrt(h/g) + transitionTime) / dt;

      if(!this->nextCFRTask_) {
        this->nextCFRTask_ = std::make_shared<prioritized_qp::Task>();
        this->nextCFRTask_->name() = "nextCFRTask";
      }

      this->nextCFRTask_->A().resize(0,2);
      this->nextCFRTask_->C() = DCM_M;
      this->nextCFRTask_->dl() = DCM_l;
      this->nextCFRTask_->du() = DCM_u;
      this->nextCFRTask_->wc() = cnoid::VectorX::Ones(DCM_l.rows());
      this->nextCFRTask_->w() = cnoid::VectorX::Ones(2) * 1e-6;
      this->nextCFRTask_->toSolve() = true;
      this->nextCFRTask_->solver().settings()->setVerbosity(debugLevel);

      tasks.push_back(this->nextCFRTask_);
    }

    bool refCOM_exist = false;
    cnoid::Vector3 refCOM = cnoid::Vector3::Zero();
    if( primitiveStates.primitiveState().find("com") != primitiveStates.primitiveState().end() && primitiveStates.primitiveState().find("com")->second->poseFollowGain().head<2>().norm() != 0.0){
      refCOM_exist = true;
      refCOM = primitiveStates.primitiveState().find("com")->second->targetPose().translation();
    }

    if(refCOM_exist){
      if(!this->refCOMTask_) {
        this->refCOMTask_ = std::make_shared<prioritized_qp::Task>();
        this->refCOMTask_->name() = "refCOMTask";
      }

      this->refCOMTask_->A() = Eigen::SparseMatrix<double,Eigen::RowMajor>(2,2);
      this->refCOMTask_->A().setIdentity();
      this->refCOMTask_->b() = refCOM.head<2>() - prevCOMCom.head<2>();
      this->refCOMTask_->C().resize(0,2);
      this->refCOMTask_->wa() = cnoid::VectorX::Ones(2);
      this->refCOMTask_->w() = cnoid::VectorX::Ones(2) * 1e-6;
      this->refCOMTask_->toSolve() = true;
      this->refCOMTask_->solver().settings()->setVerbosity(debugLevel);

      tasks.push_back(this->refCOMTask_);
    }else{
      cnoid::VectorX DCM_l;
      cnoid::VectorX DCM_u;
      Eigen::SparseMatrix<double,Eigen::RowMajor> DCM_M;
      if(Mnext.rows()>0) {
        DCM_l = lnext - Mnext * prevCOMCom.head<2>() + cnoid::VectorX::Ones(lnext.rows()) * regionMargin;
        DCM_u = unext - Mnext * prevCOMCom.head<2>() - cnoid::VectorX::Ones(unext.rows()) * regionMargin;
        DCM_M = Mnext * (sqrt(h/g) + transitionTime) / dt;
      }else{
        DCM_l = l - M * prevCOMCom.head<2>() + cnoid::VectorX::Ones(l.rows()) * regionMargin;
        DCM_u = u - M * prevCOMCom.head<2>() - cnoid::VectorX::Ones(u.rows()) * regionMargin;
        DCM_M = M * sqrt(h/g) / dt;
      }

      if(!this->stabilityMarginTask0_) {
        this->stabilityMarginTask0_ = std::make_shared<prioritized_qp::Task>();
        this->stabilityMarginTask0_->name() = "stabilityMarginTask0";
      }
      this->stabilityMarginTask0_->A().resize(0,2);
      this->stabilityMarginTask0_->C() = DCM_M;
      this->stabilityMarginTask0_->dl() = DCM_l;
      this->stabilityMarginTask0_->du() = DCM_u;
      this->stabilityMarginTask0_->wc() = cnoid::VectorX::Ones(DCM_l.rows());
      this->stabilityMarginTask0_->w() = cnoid::VectorX::Ones(2) * 1e-6;
      this->stabilityMarginTask0_->toSolve() = false;
      this->stabilityMarginTask0_->solver().settings()->setVerbosity(debugLevel);
      this->stabilityMarginTask0_->id_ext().resize(1);
      this->stabilityMarginTask0_->id_ext()[0] = "maxerror";
      this->stabilityMarginTask0_->A_ext().resize(0,1);
      this->stabilityMarginTask0_->C_ext().resize(this->stabilityMarginTask0_->C().rows(),1);
      for(int i=0;i<this->stabilityMarginTask0_->C_ext().rows();i++) this->stabilityMarginTask0_->C_ext().coeffRef(i,0) = 1.0;
      this->stabilityMarginTask0_->w_ext() = cnoid::VectorX::Ones(1) * 1e-6;
      tasks.push_back(this->stabilityMarginTask0_);

      if(!this->stabilityMarginTask1_) {
        this->stabilityMarginTask1_ = std::make_shared<prioritized_qp::Task>();
        this->stabilityMarginTask1_->name() = "stabilityMarginTask1";
      }
      this->stabilityMarginTask1_->A() = Eigen::SparseMatrix<double,Eigen::RowMajor>(DCM_M.rows()+1,2);
      this->stabilityMarginTask1_->b().resize(DCM_M.rows()+1);
      this->stabilityMarginTask1_->A().topRows(DCM_M.rows()) = DCM_M;
      this->stabilityMarginTask1_->b().head(DCM_M.rows()) = DCM_u - cnoid::VectorX::Ones(DCM_u.rows());
      this->stabilityMarginTask1_->b()[DCM_M.rows()] = 1.0;//1.0m. DCM_lは-1e10などになっていて、DCM_uのみが重要であるという前提
      this->stabilityMarginTask1_->wa() = cnoid::VectorX::Ones(DCM_M.rows()+1); // 2乗和最大化
      this->stabilityMarginTask1_->wa()[DCM_M.rows()] = 10.0; // 最小値最大化
      this->stabilityMarginTask1_->C().resize(0,2);
      this->stabilityMarginTask1_->w() = cnoid::VectorX::Ones(2) * 1e-6;
      this->stabilityMarginTask1_->toSolve() = true;
      this->stabilityMarginTask1_->solver().settings()->setVerbosity(debugLevel);
      this->stabilityMarginTask1_->id_ext().resize(1);
      this->stabilityMarginTask1_->id_ext()[0] = "maxerror";
      this->stabilityMarginTask1_->A_ext() = Eigen::SparseMatrix<double,Eigen::RowMajor>(DCM_M.rows()+1,1);
      this->stabilityMarginTask1_->A_ext().coeffRef(DCM_M.rows(),0) = 1.0;
      this->stabilityMarginTask1_->C_ext().resize(0,1);
      this->stabilityMarginTask1_->w_ext() = cnoid::VectorX::Ones(1) * 1e-6;
      tasks.push_back(this->stabilityMarginTask1_);
    }

    cnoid::VectorX result;
    if(prioritized_qp_base::solve(tasks, result, debugLevel)){
      prevCOMCom.head<2>() += result.head<2>();
    }

    l += cnoid::VectorX::Ones(l.rows()) * regionMargin;
    u -= cnoid::VectorX::Ones(u.rows()) * regionMargin;

    // prevCOMComがCFRの範囲内になるようにCFR側を直す
    cnoid::VectorX com_l = l - M * prevCOMCom.head<2>();
    cnoid::VectorX com_u = u - M * prevCOMCom.head<2>();
    for(int i=0;i<com_l.size();i++){
      if(com_l[i] > 0.0) l[i] -= com_l[i];
      if(com_u[i] < 0.0) u[i] -= com_u[i];
    }

    if( primitiveStates.primitiveState().find("com") != primitiveStates.primitiveState().end()){
      prevCOMCom[2] = primitiveStates.primitiveState().find("com")->second->targetPose().translation()[2];
    }

  }

  bool SCFRController::calcCurrentCFR(const primitive_motion_level_tools::PrimitiveStates& primitiveStates,
                                      double m,
                                      cfr_calculator::CFRCalculator& currentCFRCalculator,
                                      Eigen::SparseMatrix<double,Eigen::RowMajor>& M,
                                      cnoid::VectorX& l,
                                      cnoid::VectorX& u,
                                      std::vector<Eigen::Vector2d>& vertices,
                                      int debugLevel){
    std::vector<std::shared_ptr<primitive_motion_level_tools::PrimitiveState> > supportEEFs;
    for(std::map<std::string, std::shared_ptr<primitive_motion_level_tools::PrimitiveState> >::const_iterator it = primitiveStates.primitiveState().begin(); it != primitiveStates.primitiveState().end(); it++) {
      if(it->first != "com" && it->second->supportCOM()){
        supportEEFs.push_back(it->second);
      }
    }

    if(supportEEFs.size() == 0 || !currentCFRCalculator.computeCFR(supportEEFs, m, debugLevel)) {
      M.resize(0,2);
      l.resize(0);
      u.resize(0);
      vertices.resize(0);
      return false;
    }

    M = currentCFRCalculator.M();
    l = currentCFRCalculator.l();
    u = currentCFRCalculator.u();
    vertices = currentCFRCalculator.vertices();
    return true;
  }

  bool SCFRController::calcNextCFR(const primitive_motion_level_tools::PrimitiveStatesSequence& previewPrimitiveStates,
                                   double m,
                                   cfr_calculator::CFRCalculator& nextCFRCalculator,
                                   Eigen::SparseMatrix<double,Eigen::RowMajor>& M,
                                   cnoid::VectorX& l,
                                   cnoid::VectorX& u,
                                   std::vector<Eigen::Vector2d>& vertices,
                                   int debugLevel){
    if(previewPrimitiveStates.primitiveStates().size() == 0) {
      M.resize(0,2);
      l.resize(0);
      u.resize(0);
      vertices.resize(0);
      return false;
    }

    std::vector<std::shared_ptr<primitive_motion_level_tools::PrimitiveState> > supportEEFs;
    for(std::map<std::string, std::shared_ptr<primitive_motion_level_tools::PrimitiveState> >::const_iterator it = previewPrimitiveStates.primitiveStates()[0]->primitiveState().begin(); it != previewPrimitiveStates.primitiveStates()[0]->primitiveState().end(); it++) {
      if(it->first != "com" && it->second->supportCOM()){
        supportEEFs.push_back(it->second);
      }
    }

    if(supportEEFs.size() == 0 || !nextCFRCalculator.computeCFR(supportEEFs, m, debugLevel)) {
      M.resize(0,2);
      l.resize(0);
      u.resize(0);
      vertices.resize(0);
      return false;
    }

    M = nextCFRCalculator.M();
    l = nextCFRCalculator.l();
    u = nextCFRCalculator.u();
    vertices = nextCFRCalculator.vertices();
    return true;
  }

}
