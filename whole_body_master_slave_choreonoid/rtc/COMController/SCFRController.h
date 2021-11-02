#ifndef WHOLEBODYMASTERSLAVECHOREONOID_COMCONTROLLER_SCFRCONTROLLER_H
#define WHOLEBODYMASTERSLAVECHOREONOID_COMCONTROLLER_SCFRCONTROLLER_H

#include <primitive_motion_level_tools/PrimitiveState.h>
#include <cnoid/Body>

namespace whole_body_master_slave_choreonoid{
  class SCFRController {
  public:
    void control(const primitive_motion_level_tools::PrimitiveStates& primitiveStates,
                 const primitive_motion_level_tools::PrimitiveStatesSequence& previewPrimitiveStates,
                 double m,
                 double dt,
                 double regionMargin_,
                 cnoid::Vector3& prevCOMCom,
                 Eigen::SparseMatrix<double,Eigen::RowMajor>& M,
                 cnoid::VectorX& l,
                 cnoid::VectorX& u,
                 std::vector<Eigen::Vector2d>& vertices,
                 std::vector<Eigen::Vector2d>& previewVertices);
  protected:
  };
}

#endif
