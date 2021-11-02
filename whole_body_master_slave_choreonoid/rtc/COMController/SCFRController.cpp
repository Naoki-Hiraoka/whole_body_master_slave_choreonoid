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

    SCFRController::calcCurrentCFR(primitiveStates,m,this->currentCFRCalculator_,M,l,u,vertices,debugLevel);

    Eigen::SparseMatrix<double,Eigen::RowMajor> Mnext;
    cnoid::VectorX lnext;
    cnoid::VectorX unext;
    SCFRController::calcNextCFR(previewPrimitiveStates,m,this->nextCFRCalculator_,Mnext,lnext,unext,previewVertices,debugLevel);

    double transitionTime = dt;
    if(previewPrimitiveStates.primitiveStates().size() > 0 && previewPrimitiveStates.primitiveStates()[0]->time() > dt) transitionTime = previewPrimitiveStates.primitiveStates()[0]->time();

    /*
      下記優先順位
      速度・加速度のCurrentCFR上下限を満たす
      transitionTime後に速度のNextCFR上下限を満たす
      CurrentCFRをregionMarginで満たす
      transitionTime後にNextCFRをregionMarginで満たす
      COMRefに追従する
    */
    //cnoid::Vector3 COMRef = 
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
