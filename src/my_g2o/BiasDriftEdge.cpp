// #include <pose_graph_mapping/custom_types/gps_edge.h>
// #include <pose_graph_mapping/custom_types/robot_gps.h>
// #include "include/GPSEdge.h"
#include <iostream>
#include "BiasDriftEdge.h"

using namespace std;
using namespace Eigen;

namespace g2o
{

  BiasDriftEdge::BiasDriftEdge() : BaseMultiEdge<1, Eigen::Matrix<double,1,1>>() //sateliite position + pseudorange
  {
  // resizeParameters(1);
    resize(3); // Resized number of vertexes
    // _information.resize(2, 2);
		// _error.resize(3, 1);
		// _measurement.resize(2, 1);
		// _dimension = 3;
  }

  void BiasDriftEdge::computeError()
  {
    const BiasVertex *v1 = static_cast<const BiasVertex *>(_vertices[0]); // Receiver
    const BiasVertex *v2 = static_cast<const BiasVertex *>(_vertices[1]); // Receiver
    const BiasDriftVertex *v3 = static_cast<const BiasDriftVertex *>(_vertices[2]); // Satellite
    
    double bias_prev = v1->estimate()[0];
    double bias_act = v2->estimate()[0];
    double time = _measurement[0];
    double biasDrift = v3->estimate()[0];

    _error[0] =  (bias_act - bias_prev) / time - biasDrift;

  }

  void BiasDriftEdge::setInformation(const double &i)
  {
    _information(0,0) = i;
  }

  void BiasDriftEdge::setMeasurement( Eigen::Matrix<double,1,1> measurement)
  {
    _measurement = measurement;
  }

  bool BiasDriftEdge::read(std::istream &is)
  {

    for (unsigned int i = 0; i < 1; i++)
      is >> _measurement[i];

    is >> _information(0,0);
    return true;
  }

  bool BiasDriftEdge::write(std::ostream &os) const
  {
    for (unsigned int i = 0; i < 1; i++)
      os << std::setprecision(15) << _measurement[i] << " ";

    // write information matrix
    os << information()(0,0);

    return os.good();
  }
} // end namespace