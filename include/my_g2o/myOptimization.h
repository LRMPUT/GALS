#ifndef MY_OPTIMIZATION_H
#define MY_OPTIMIZATION_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <cstdio>

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
#include "DistanceEdge.h"

G2O_USE_TYPE_GROUP(slam2d);
G2O_USE_TYPE_GROUP(slam3d);
G2O_USE_OPTIMIZATION_LIBRARY(dense);

class LaserPose
{
    friend class MyOptimization;
    LaserPose(Eigen::Affine3d pose_, int week_, double tow_)
    {
        pose = pose_;
        week = week_;
        tow = tow_;
    }
    double getTow() {return tow;}
    Eigen::Affine3d getPose() {return pose;}
    private:
    Eigen::Affine3d pose;
    int week;
    double tow;
};

class OptimizationResults
{
    friend class MyOptimization;
    OptimizationResults(int roverId, Eigen::Matrix4d estPose, std::array<double,5> lastBiases, int week_, double tow_)
    {
        roverVertexId = roverId;
        estimatedRoverPose = estPose;

        for (int i = 0; i < 5; i++){
            biasVertexId[i] = roverId + i + 1;
            estimatedBiasValue[i] = lastBiases[i];
        }
        week = week_;
        tow = tow_;
    }
    Eigen::Matrix4d getEstimatedRoverPose() { return estimatedRoverPose; }
    std::array<double,5> getEstimatedBiasValue() { return estimatedBiasValue; }
    double getTow() {return tow;}
    int getWeek() {return week;}
    int getRoverVertexId() { return roverVertexId; }
    private:
    int roverVertexId;
    Eigen::Matrix4d estimatedRoverPose;
    std::array<int,5> biasVertexId;
    std::array<double,5> estimatedBiasValue;
    // Instead of timestamp
    int week;
    double tow;
};

class MyOptimization
{
    public:
    MyOptimization( bool verbose, int iter);
    void addRoverVertex(const Eigen::Vector3d &est);
    void addBiasesVertices(const std::array<double,5> &est);
    void addEdgeSatPrior(Eigen::Matrix<double, 4, 1> &measurement, double information, int sys);
    void addLaserEdge(int, double);
    Eigen::Vector3d getLastRoverPose();
    std::array<double,5> getLastBiasesValue();
    void optimize();
    void optimizeAll();
    void processOutput(int, double);
    void saveOutputToFile(std::string filename, Eigen::Matrix4d pose, int week, double tow);
    bool readLaserData(std::string);

    private:

    g2o::SparseOptimizer optimizer;
    int lastVertexId;
    int maxIterations;
    int optLevel;   // Each new rover vertex starts new level
    int numBiases;
    Eigen::Vector3d lastRoverPose;
    std::array<double,5> lastBiasesValue;
    std::vector<OptimizationResults> optimizationResults;
    std::vector<LaserPose> laserPoses;
};

#endif