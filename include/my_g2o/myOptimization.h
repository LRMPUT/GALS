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
#include "g2o/types/slam3d/vertex_pointxyz.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/slam3d/edge_se3_xyzprior.h"
#include "g2o/types/slam3d/edge_se3_prior.h"
#include "g2o/core/sparse_optimizer_terminate_action.h"
#include "../RTKLIB/src/rtklib.h"

#include "GPSEdge.h"
#include "GPSEdgePrior.h"
#include "BiasVertex.h"
#include "DistanceEdge.h"
#include "myParameters.h"
#include "BiasDriftVertex.h"
#include "BiasDriftEdge.h"
#include "DopplerEdge.h"

#include <ros/ros.h>
#include <ros/package.h>

G2O_USE_TYPE_GROUP(slam2d);
G2O_USE_TYPE_GROUP(slam3d);
G2O_USE_OPTIMIZATION_LIBRARY(dense);

#define NUMBIASES 5
#define NUMDRIFTBIASES 0


class LaserPose
{
    friend class MyOptimization;
    public:
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
    OptimizationResults(int roverId, Eigen::Matrix4d estPose, std::array<double,NUMBIASES> lastBiases, int week_, double tow_)
    {
        roverVertexId = roverId;
        estimatedRoverPose = estPose;

        for (int i = 0; i < NUMBIASES; i++){
            biasVertexId[i] = roverId + i + 1;
            estimatedBiasValue[i] = lastBiases[i];
        }
        week = week_;
        tow = tow_;
        velocity[0] = velocity[1] = velocity[2] = 0;
    }
    Eigen::Matrix4d getEstimatedRoverPose() { return estimatedRoverPose; }
    std::array<double,NUMBIASES> getEstimatedBiasValue() { return estimatedBiasValue; }
    double getTow() {return tow;}
    int getWeek() {return week;}
    int getRoverVertexId() { return roverVertexId; }
    void setVelocity(std::array<double,3> vel) { velocity = vel; }
    std::array<double,3> getVelocity() {return velocity; }
    private:
    int roverVertexId;
    Eigen::Matrix4d estimatedRoverPose;
    std::array<int,NUMBIASES> biasVertexId;
    std::array<double,NUMBIASES> estimatedBiasValue;
    std::array<double,3> velocity;
    // Instead of timestamp
    int week;
    double tow;
};

class MyOptimization
{
    public:
    MyOptimization();
    void addBiasDriftVertex();
    void addBiasDriftEdge(int, double);
    void addRoverVertex(const Eigen::Matrix4d &est);
    void addBiasesVertices(const std::array<double,NUMBIASES> &est);
    void addEdgeSatPrior(Eigen::Matrix<double, 4, 1> &measurement, double information, int sys);
    void addLaserEdge(int, double, Eigen::Vector3d);
    void addDopplerEdge(int, double);
    void filterGPS(Eigen::Vector3d libPose, double tow, int &stat);
    Eigen::Matrix4d getLastRoverPose();
    std::array<double,NUMBIASES> getLastBiasesValue();
    void optimize();
    void optimizeAll();
    void processOutput(int, double);
    void saveOutputToFile(std::string filename, const Eigen::Matrix4d pose, int week, double tow);
    void saveBiasesToFile(std::string filename, std::array<double,NUMBIASES> lastBiasesValue, int week, double tow);
    bool readLaserData(std::string);
    void addVelToLastOptimResult(std::array <double,3> vel, std::array <double,3> cov, double tow);
    void saveG2OFile();

    private:

    g2o::SparseOptimizer optimizer;
    int lastVertexId;
    int maxIterations;
    int optLevel;   // Each new rover vertex starts new level
    int numBiases;
    Eigen::Matrix4d lastRoverPose;
    std::array<double,NUMBIASES> lastBiasesValue;
    std::vector<OptimizationResults> optimizationResults;
    std::vector<LaserPose> laserPoses;
    int firstPoseWithLaserEdgeId;
    std::vector<g2o::EdgeSE3*> laserEdgesList;
    std::vector<g2o::DopplerEdge*> dopplerEdgesList;
    std::vector<std::vector<g2o::GPSEdgePrior*>> GPSEdgesListList;
    std::vector<g2o::BiasVertex*> biasList;
    std::vector<g2o::BiasDriftEdge*> biasDriftEdgesList;
    std::array<double, 3> actDopplVel;
    std::array<double, 3> actDopplCov;
    double actDopplTow;
};

#endif