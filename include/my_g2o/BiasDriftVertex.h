#ifndef G2O_BIAS_DRIFT_VERTEX_H
#define G2O_BIAS_DRIFT_VERTEX_H

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

namespace g2o
{

  class BiasDriftVertex : public BaseVertex<1, Eigen::Matrix<double, 1, 1, Eigen::ColMajor>>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    BiasDriftVertex();

    virtual void setToOriginImpl()
    {
      _estimate.setZero();
    }

    virtual bool setEstimateDataImpl(const number_t *est)
    {
      _estimate[0] = est[0];
      return true;
    }

    virtual bool getEstimateData(number_t *est) const
    {
      est[0] = _estimate[0];
      return true;
    }

    virtual int estimateDimension() const
    {
      return 1;
    }

    virtual void oplusImpl(const number_t *update)
    {
      _estimate[0] += update[0];
    }

    virtual bool setMinimalEstimateDataImpl(const number_t *est)
    {
      return setEstimateData(est);
    }

    virtual bool getMinimalEstimateData(number_t *est) const
    {
      return getEstimateData(est);
    }

    virtual int minimalEstimateDimension() const
    {
      return 1;
    }

    virtual bool read(std::istream & /*is*/);
    virtual bool write(std::ostream & /*os*/) const;
  };
}
#endif