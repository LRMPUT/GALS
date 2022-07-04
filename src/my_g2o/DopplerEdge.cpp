// #include <pose_graph_mapping/custom_types/gps_edge.h>
// #include <pose_graph_mapping/custom_types/robot_gps.h>
// #include "include/GPSEdge.h"
#include <iostream>
#include "DopplerEdge.h"

using namespace std;
using namespace Eigen;

namespace g2o
{

  DopplerEdge::DopplerEdge() : BaseBinaryEdge<3, std::array<double,3>, VertexSE3, VertexSE3>() // Measurement is distance constraint in 3 axes
  {
  }

  void DopplerEdge::computeError()
  {

    const VertexSE3 *v1 = static_cast<const VertexSE3 *>(_vertices[0]); // previous pose
    const VertexSE3 *v2 = static_cast<const VertexSE3 *>(_vertices[1]); // actual pose
    
    _error[0] =  v2->estimate().translation()[0] - v1->estimate().translation()[0] - _measurement[0] / 1e2;
    _error[1] =  v2->estimate().translation()[1] - v1->estimate().translation()[1] - _measurement[1] / 1e2;
    _error[2] =  v2->estimate().translation()[2] - v1->estimate().translation()[2] - _measurement[2] / 1e2;
  }

  void DopplerEdge::setInformation(const std::array<double,3> &  inf)
  {
    _information = Eigen::Matrix3d::Identity();
    _information(0,0) = inf[0];
    _information(1,1) = inf[1];
    _information(2,2) = inf[2];
  }

  void DopplerEdge::setMeasurement(std::array<double,3> measurement)
  {
    _measurement = measurement;
  }

  bool DopplerEdge::read(std::istream &is)
  {

    for (unsigned int i = 0; i < 3; i++)
      is >> _measurement[i];

  for (unsigned int i = 0; i < 3; i++)
    is >> _information(i,i);
    return true;
  }

  bool DopplerEdge::write(std::ostream &os) const
  {
    for (unsigned int i = 0; i < 3; i++)
      os << std::setprecision(15) << _measurement[i] << " ";

    // write information matrix
    for (unsigned int i = 0; i < 3; i++)
      os << information()(0, 0);

    return os.good();
  }
} // end namespace