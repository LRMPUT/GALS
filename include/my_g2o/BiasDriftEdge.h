#ifndef G2O_BIAS_DRIFT_EDGE_H
#define G2O_BIAS_DRIFT_EDGE_H

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
#include "BiasVertex.h"
#include "BiasDriftVertex.h"

namespace g2o{

// E - meausrement type, D - error/information dimenstion
class G2O_TYPES_DATA_API BiasDriftEdge: public BaseMultiEdge<1, Eigen::Matrix<double,1,1>>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    BiasDriftEdge();

    void computeError();

    virtual bool read(std::istream& /*is*/);
    virtual bool write(std::ostream& /*os*/) const;


    virtual void setInformation(const double& i);
    virtual void setMeasurement(Eigen::Matrix<double,1,1> measurement);

  };
}
#endif