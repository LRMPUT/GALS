// #include <pose_graph_mapping/custom_types/gps_edge.h>
// #include <pose_graph_mapping/custom_types/robot_gps.h>
// #include "include/GPSEdge.h"
#include <iostream>
#include "GPSEdgePrior.h"
#include "BiasVertex.h"

using namespace std;
using namespace Eigen;

namespace g2o
{

  GPSEdgePrior::GPSEdgePrior() : BaseMultiEdge<1, Eigen::Matrix<double,4,1>>() //sateliite position + pseudorange
  {
    resize(2); // Resized number of vertexes
  }

  void GPSEdgePrior::computeError()
  {
    const VertexSE3 *v1 = static_cast<const VertexSE3 *>(_vertices[0]); // Receiver
    const BiasVertex *v2 = static_cast<const BiasVertex *>(_vertices[1]); // Satellite
    
    Vector3d satPos = Eigen::Vector3d(_measurement[0], _measurement[1], _measurement[2]);
    satPos = satPos / 1e2;
    double prng = _measurement[3]/ 1e2;
    double bias = v2->estimate()[0];

    Vector3d t = v1->estimate().translation() - satPos;
    double current_dist = t.norm() + 1e2 * OMGE * (satPos[0] * v1->estimate().translation()[1] - satPos[1] * v1->estimate().translation()[0]) / CLIGHT;
    _error[0] =  current_dist - prng + bias;


  }

  void GPSEdgePrior::setInformation(const double &i)
  {
    _information(0,0) = i;
  }

  void GPSEdgePrior::setMeasurement( Eigen::Matrix<double,4,1> measurement)
  {
    _measurement = measurement;
  }

  bool GPSEdgePrior::read(std::istream &is)
  {

    for (unsigned int i = 0; i < 4; i++)
      is >> _measurement[i];

    is >> _information(0,0);
    return true;
  }

  bool GPSEdgePrior::write(std::ostream &os) const
  {
    for (unsigned int i = 0; i < 4; i++)
      os << std::setprecision(15) << _measurement[i] << " ";

    // write information matrix
    os << information()(0,0);

    return os.good();
  }
} // end namespace