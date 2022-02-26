#include <iostream>
#include "BiasDriftVertex.h"

using namespace std;
using namespace Eigen;

namespace g2o
{

  BiasDriftVertex::BiasDriftVertex() : BaseVertex<1, Eigen::Matrix<double, 1, 1, Eigen::ColMajor>>()
  {
        _estimate.setZero(); 
  }

  bool BiasDriftVertex::read(std::istream &is)
  {
    is >> _estimate[0];
    return true;
  }

  bool BiasDriftVertex::write(std::ostream &os) const
  {
    os << std::setprecision(15) << estimate()(0);
    return true;
  }
} // end namespace