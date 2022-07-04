// #include <pose_graph_mapping/custom_types/gps_edge.h>
// #include <pose_graph_mapping/custom_types/robot_gps.h>
// #include "include/GPSEdge.h"
#include <iostream>
#include "GPSEdge.h"
#include "BiasVertex.h"

using namespace std;
using namespace Eigen;

double haversine(double lat1, double long1, double lat2, double long2)
{
  double dlong = (long2 - long1) * d2r;
  double dlat = (lat2 - lat1) * d2r;
  double a = pow(sin(dlat / 2.0), 2) + cos(lat1) * cos(lat2) * pow(sin(dlong / 2.0), 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  double d = EARTH_RADIUS * c;

  return d;
}

namespace g2o
{
  double GPSEdge::bias;

  GPSEdge::GPSEdge() :BaseMultiEdge<1, Eigen::Vector2d>()
  {
    resize(3); // Resized number of vertexes
  }

  void GPSEdge::computeError()
  {
    const VertexSE3 *v1 = static_cast<const VertexSE3 *>(_vertices[0]); // Receiver
    const VertexSE3 *v2 = static_cast<const VertexSE3 *>(_vertices[1]); // Satellite
    const BiasVertex *v3 = static_cast<const BiasVertex *>(_vertices[2]); // Bias
    Vector3d t = v1->estimate().translation() - v2->estimate().translation();

    double current_dist = t.norm() +  OMGE*(v2->estimate().translation()[0]*v1->estimate().translation()[1]-v2->estimate().translation()[1]*v1->estimate().translation()[0])/CLIGHT;;
    double bias = v3->estimate()[0];
    _error[0] = current_dist  - _measurement[0] + bias; //

  }

  void GPSEdge::setInformation(const double &i)
  {
    _information(0,0) = i;
  }

  void GPSEdge::setMeasurement(double distance)
  {
    _measurement[0] = distance;
    // _measurement[1] = 100;
  }

  bool GPSEdge::read(std::istream &is)
  {
    // double distance, information;
    // is >> distance;
    // is >> information;
    // setMeasurement(distance);
    // setInformation(information);
    // return is.good() || is.eof();
  }

  bool GPSEdge::write(std::ostream &os) const
  {
  //     os << _measurement << " " << _information << " ";
  //     return os.good();
  }
} // end namespace