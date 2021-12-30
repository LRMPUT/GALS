#include "DistanceEdge.h"

using namespace std;
using namespace Eigen;

namespace g2o
{
  DistanceEdge::DistanceEdge() :BaseBinaryEdge<1, double, VertexSE3, VertexSE3>()
  {
  // resizeParameters(1);
  }

  void DistanceEdge::computeError()
  {
    const VertexSE3 *v1 = static_cast<const VertexSE3 *>(_vertices[0]); // Receiver
    const VertexSE3 *v2 = static_cast<const VertexSE3 *>(_vertices[1]); // Satellite
    Vector3d t = v1->estimate().translation() - v2->estimate().translation();

    double current_dist = t.norm();
    _error[0] = current_dist  - _measurement; // - biass
  }

  void DistanceEdge::setInformation(const double &i)
  {
    _information[0] = i;
  }

  void DistanceEdge::setMeasurement(double distance)
  {
    _measurement = distance;
  }

  bool DistanceEdge::read(std::istream &is)
  {
  }

  bool DistanceEdge::write(std::ostream &os) const
  {
  }
} // end namespace