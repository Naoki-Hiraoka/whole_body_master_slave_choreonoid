#ifndef WHOLEBODYMASTERSLAVECHOREONOID_COMCONTROLLER_SCFRCONTROLLER_H
#define WHOLEBODYMASTERSLAVECHOREONOID_COMCONTROLLER_SCFRCONTROLLER_H

#include <primitive_motion_level_tools/PrimitiveState.h>
#include <cnoid/Body>
#include <cfr_calculator/cfr_calculator.h>

namespace whole_body_master_slave_choreonoid{
  class SCFRController {
  public:
    void control(const primitive_motion_level_tools::PrimitiveStates& primitiveStates,
                 const primitive_motion_level_tools::PrimitiveStatesSequence& previewPrimitiveStates,
                 double m,
                 double dt,
                 double regionMargin_,
                 cnoid::Vector3& prevCOMCom, // inout & output
                 Eigen::SparseMatrix<double,Eigen::RowMajor>& M, // output
                 cnoid::VectorX& l, // output
                 cnoid::VectorX& u, // output
                 std::vector<Eigen::Vector2d>& vertices, // output
                 std::vector<Eigen::Vector2d>& previewVertices, // output
                 int debugLevel
                 );
  protected:
    cfr_calculator::CFRCalculator currentCFRCalculator_;
    cfr_calculator::CFRCalculator nextCFRCalculator_;

    static bool calcCurrentCFR(const primitive_motion_level_tools::PrimitiveStates& primitiveStates,
                               double m,
                               cfr_calculator::CFRCalculator& currentCFRCalculator,
                               Eigen::SparseMatrix<double,Eigen::RowMajor>& M,
                               cnoid::VectorX& l,
                               cnoid::VectorX& u,
                               std::vector<Eigen::Vector2d>& vertices,
                               int debugLevel);
    static bool calcNextCFR(const primitive_motion_level_tools::PrimitiveStatesSequence& previewPrimitiveStates,
                            double m,
                            cfr_calculator::CFRCalculator& nextCFRCalculator,
                            Eigen::SparseMatrix<double,Eigen::RowMajor>& M,
                            cnoid::VectorX& l,
                            cnoid::VectorX& u,
                            std::vector<Eigen::Vector2d>& vertices,
                            int debugLevel);
  };
}

#endif
