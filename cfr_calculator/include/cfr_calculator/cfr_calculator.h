#ifndef CFRCALCULTOR_CFRCALCULTOR_H
#define CFRCALCULTOR_CFRCALCULTOR_H

#include <map>
#include <memory>
#include <Eigen/Eigen>
#include <vector>
#include <primitive_motion_level_tools/PrimitiveState.h>

namespace cfr_calculator {

  class CFRCalculator {
  public:
    bool computeCFR(const std::vector<std::shared_ptr<primitive_motion_level_tools::PrimitiveState> >& supportEEFs,
                    double m/*robotの質量*/,
                    int debugLevel);
    const Eigen::SparseMatrix<double,Eigen::RowMajor>& M() const { return M_;}
    const Eigen::VectorXd& l() const { return l_;}
    const Eigen::VectorXd& u() const { return u_;}
    const std::vector<Eigen::Vector2d>& vertices() const { return vertices_;}
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  protected:
    Eigen::SparseMatrix<double,Eigen::RowMajor> M_ = Eigen::SparseMatrix<double,Eigen::RowMajor>(0,2);
    Eigen::VectorXd l_;
    Eigen::VectorXd u_;
    std::vector<Eigen::Vector2d> vertices_;
  };

}

#endif
