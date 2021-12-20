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

  GPSEdgePrior::GPSEdgePrior() :BaseMultiEdge<1, Eigen::Matrix<double,4,1>>() //sateliite position + pseudorange
  {
  // resizeParameters(1);
    resize(2); // Resized number of vertexes
    // _information.resize(2, 2);
		// _error.resize(3, 1);
		// _measurement.resize(2, 1);
		// _dimension = 3;
  }

  void GPSEdgePrior::computeError()
  {
    const VertexSE3 *v1 = static_cast<const VertexSE3 *>(_vertices[0]); // Receiver
    const BiasVertex *v2 = static_cast<const BiasVertex *>(_vertices[1]); // Satellite
    
    Vector3d satPos = Eigen::Vector3d(_measurement[0], _measurement[1], _measurement[2]);
    double prng = _measurement[3];
    double bias = v2->estimate()[0];

    Vector3d t = v1->estimate().translation() - satPos;
    double current_dist = t.norm() + OMGE * (satPos[0] * v1->estimate().translation()[1] - satPos[1] * v1->estimate().translation()[0]) / CLIGHT;

    _error[0] = current_dist  - prng + bias;

  }

  void GPSEdgePrior::setInformation(const double &i)
  {
    _information[0] = i;
  }

  void GPSEdgePrior::setMeasurement( Eigen::Matrix<double,4,1> measurement)
  {
    _measurement = measurement;
  }

  bool GPSEdgePrior::read(std::istream &is)
  {
  }

  bool GPSEdgePrior::write(std::ostream &os) const
  {
  }
} // end namespace