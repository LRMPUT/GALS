#ifndef MY_OPTIMIZATION_H
#define MY_OPTIMIZATION_H

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
#include "g2o/types/slam3d/GPSEdge.h"
#include "g2o/core/sparse_optimizer_terminate_action.h"
#include "../RTKLIB/src/rtklib.h"

#include "GPSEdge.h"
#include "GPSEdgePrior.h"
#include "BiasVertex.h"

G2O_USE_TYPE_GROUP(slam2d);
G2O_USE_TYPE_GROUP(slam3d);
G2O_USE_OPTIMIZATION_LIBRARY(dense);

class MyOptimization
{
    public:

    MyOptimization( bool verbose, int iter);
    void addRoverVertex(const Eigen::Vector3d &est);
    void addBiasesVertices(const std::vector<double> &est);
    void addEdgeSatPrior(Eigen::Matrix<double, 4, 1> &measurement, double information, int sys);
    Eigen::Vector3d getLastRoverPose();
    std::vector<double> getLastBiasesValue();
    void optimize();
    void processOutput(int, double);

    private:

    g2o::SparseOptimizer optimizer;
    Eigen::Vector3d lastRoverPose;
    int numBiases;
    std::vector<double> lastBiasesValue;
    int lastVertexId;
    int maxIterations;
};

#endif