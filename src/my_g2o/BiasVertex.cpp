#include <iostream>
#include "BiasVertex.h"

using namespace std;
using namespace Eigen;

namespace g2o
{

  BiasVertex::BiasVertex() : BaseVertex<1, Eigen::Matrix<double, 1, 1, Eigen::ColMajor>>()
  {
        _estimate.setZero(); 
  }

  bool BiasVertex::read(std::istream &is)
  {
    is >> _estimate[0];
    return true;
  }

  bool BiasVertex::write(std::ostream &os) const
  {
    os << std::setprecision(15) << estimate()(0);
    return true;
  }
} // end namespace