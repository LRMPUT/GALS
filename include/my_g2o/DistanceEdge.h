#ifndef G2O_DISTANCE_EDGE_H
#define G2O_DISTANCE_EDGE_H

#include <g2o/types/data/g2o_types_data_api.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/types/slam2d/vertex_se2.h>

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
#include "g2o/types/slam3d/edge_se3.h"
#include <Eigen/Core>

namespace g2o{


class G2O_TYPES_DATA_API DistanceEdge: public BaseBinaryEdge<1, double, VertexSE3, VertexSE3>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    DistanceEdge();

    void computeError();

    virtual bool read(std::istream& /*is*/);
    virtual bool write(std::ostream& /*os*/) const;

    virtual void setInformation(const double& i);
    virtual void setMeasurement(double dst);
  };
}
#endif