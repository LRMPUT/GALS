#ifndef G2O_DOPPLER_EDGE_H
#define G2O_DOPPLER_EDGE_H

#include <g2o/types/data/g2o_types_data_api.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/config.h>
#include <g2o/core/base_multi_edge.h>
#include "g2o/core/base_binary_edge.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>

#include "g2o/core/auto_differentiation.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/stuff/command_args.h"
#include "g2o/stuff/sampler.h"
#include "g2o/core/factory.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/vertex_pointxyz.h"
#include "g2o/types/slam3d/edge_se3.h"
#include <Eigen/Core>

#define d2r (M_PI / 180.0)
#define r2d (180.0 / M_PI)
#define EARTH_RADIUS 6371000 // R is earth’s radius (mean radius = 6,371km)
#define OMGE 7.2921151467E-5 // /* earth angular velocity (IS-GPS) (rad/s) */
#define CLIGHT      299792458.0         /* speed of light (m/s) */

namespace g2o{

// E - meausrement type, D - error/information dimenstion
class G2O_TYPES_DATA_API DopplerEdge: public BaseBinaryEdge<3, std::array<double,3>, VertexSE3, VertexSE3>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    DopplerEdge();

    void computeError();

    virtual bool read(std::istream& /*is*/);
    virtual bool write(std::ostream& /*os*/) const;


    virtual void setInformation(const std::array<double,3> &);
    virtual void setMeasurement(std::array<double,3> measurement);

  };
}
#endif