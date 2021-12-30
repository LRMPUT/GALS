// g2o - General Graph Optimization
// Copyright (C) 2012 R. Kümmerle
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PUasaRPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "myOptimization.h"

MyOptimization::MyOptimization(bool verbose, int iter) : numBiases(5), lastVertexId(-1), optLevel(0)
{

    // std::vector<g2o::VertexSE3*> vertices;
    // std::vector<g2o::GPSEdge*> edges;
    optimizer.setVerbose(verbose);

    g2o::SparseOptimizerTerminateAction* terminateAction = new g2o::SparseOptimizerTerminateAction;
    terminateAction->setGainThreshold(0.0001); // 0.0000001
    terminateAction->setMaxIterations(iter);
    optimizer.addPostIterationAction(terminateAction);
    // allocate the solver
    g2o::OptimizationAlgorithmProperty solverProperty;
    optimizer.setAlgorithm(g2o::OptimizationAlgorithmFactory::instance()->construct("lm_dense", solverProperty)); // GN - can start from (0,0),  LM - must have inital estimate, but works better

    lastRoverPose = Eigen::Vector3d::Zero();
    for (int i =0;i<numBiases;i++)
      lastBiasesValue[i] = 0;

    maxIterations = iter;
}
void MyOptimization::addRoverVertex(const Eigen::Vector3d &est)
{
    g2o::VertexSE3 *vertex = new g2o::VertexSE3;
    g2o::Isometry3 estIso = g2o::Isometry3::Identity();
    estIso.translation() = est;
    vertex->setEstimate(estIso);
    vertex->setId(++lastVertexId);
    // if (lastVertexId == 0)
    //   vertex->setFixed(true);
    // else
    vertex->setFixed(false);
    optimizer.addVertex(vertex);
}

void MyOptimization::addBiasesVertices(const std::array<double,5> &est)
{
  if (est.size() != numBiases)
    std::cout << "Invalid size of bias vector";

  for (int i = 0; i < numBiases; i++)
  {
    g2o::BiasVertex *bv = new g2o::BiasVertex();
    Eigen::Matrix<double, 1, 1> estMat = Eigen::Matrix<double, 1, 1>::Zero();
    estMat[0] = est[i];
    bv->setId(++lastVertexId);
    // bv->setFixed(true);
    bv->setEstimate(estMat);
    optimizer.addVertex(bv);
  }
}

void MyOptimization::addEdgeSatPrior(Eigen::Matrix<double, 4, 1> &measurement, double information, int sys)
{
  g2o::GPSEdgePrior *edgeSatPrior = new g2o::GPSEdgePrior;

  edgeSatPrior->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(lastVertexId-numBiases)));

  if      (sys == SYS_GPS){edgeSatPrior->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(lastVertexId-4)));}
  else if (sys==SYS_GLO) {edgeSatPrior->setVertex(1,  dynamic_cast<g2o::OptimizableGraph::Vertex *> (optimizer.vertex(lastVertexId-3)));}
  else if (sys==SYS_GAL) {edgeSatPrior->setVertex(1,  dynamic_cast<g2o::OptimizableGraph::Vertex *> (optimizer.vertex(lastVertexId-2)));}
  else if (sys==SYS_CMP) {edgeSatPrior->setVertex(1,  dynamic_cast<g2o::OptimizableGraph::Vertex *> (optimizer.vertex(lastVertexId-1)));} 
  else if (sys==SYS_QZS) {edgeSatPrior->setVertex(1,  dynamic_cast<g2o::OptimizableGraph::Vertex *> (optimizer.vertex(lastVertexId)));}
  // edgeSat->setVertex(2,  dynamic_cast<g2o::OptimizableGraph::Vertex *> (biases[0]));
  edgeSatPrior->setInformation(information);
  // edgeSat->setMeasurement(my_prng[j]);
  //edges.push_back(edgeSat);
  
  edgeSatPrior->setMeasurement(measurement);
  edgeSatPrior->setLevel(optLevel);
  // optimizer.addVertex(vertexSat);
  optimizer.addEdge(edgeSatPrior);
}

Eigen::Vector3d MyOptimization::getLastRoverPose()
{
  return lastRoverPose;
}

std::array<double,5> MyOptimization::getLastBiasesValue()
{
  return lastBiasesValue;
}

void MyOptimization::optimize()
{
    optimizer.initializeOptimization(optLevel++);
    optimizer.optimize(maxIterations);
}

void MyOptimization::optimizeAll()
{
    for (int i=0; i < optimizationResults.size(); i++)
      optimizer.vertex(optimizationResults[i].getRoverVertexId())->setFixed(false);

    std::cout << "Starting whole optimization" << std::endl;
    optimizer.initializeOptimization(-1);
    optimizer.optimize(30);
    std::cout << "Ended whole optimization" << std::endl;

    for (int i=0; i < optimizationResults.size(); i++)
    {
      g2o::VertexSE3 *v = static_cast<g2o::VertexSE3 *>(optimizer.vertex(optimizationResults[i].getRoverVertexId()));
      saveOutputToFile("g2o_sol_all.txt", v->estimate().matrix(), optimizationResults[i].getWeek(), optimizationResults[i].getTow());
    }

}

void MyOptimization::processOutput(int week, double tow)
{
  // for (auto it = optimizer.vertices().begin(); it != optimizer.vertices().end(); ++it)
  //    g2o::VertexSE3 *v = static_cast<g2o::VertexSE3 *>(it->second);

  // Estimated pose
  g2o::VertexSE3 *v = static_cast<g2o::VertexSE3 *>(optimizer.vertex(lastVertexId - numBiases));
  Eigen::Matrix4d estPose = v->estimate().matrix();
  lastRoverPose = Eigen::Vector3d(estPose(0, 3), estPose(1, 3), estPose(2, 3));

  // Estimated biases
  for (int i = 1; i < 6; i++)
  {
    g2o::BiasVertex *bv = static_cast<g2o::BiasVertex *>(optimizer.vertex(lastVertexId - numBiases + i));
    lastBiasesValue[i-1] = bv->estimate()[0];
  }

  // Store estimated values in separate vector
  OptimizationResults optimResult = OptimizationResults(lastVertexId - numBiases, estPose, lastBiasesValue, week, tow);
  optimizationResults.push_back(optimResult);
  // Save results to file
  saveOutputToFile("g2o_sol.txt", estPose, week, tow);
}

void MyOptimization::saveOutputToFile(std::string filename, Eigen::Matrix4d pose, int week, double tow)
{
  std::ofstream fileOut;
  static bool initalizeFile = true;
  if (initalizeFile)
  {
    remove("g2o_sol.txt");
    remove("g2o_sol_all.txt");
    // fileOut.open(filename);
    // fileOut.close();
    initalizeFile = false;
  }
  fileOut.open(filename, std::ios_base::app);
  fileOut << std::setprecision(15) << week << "," << tow << "," << pose(0, 3) << "," << pose(1, 3) << "," << pose(2, 3) << std::endl;
  fileOut.close();
}

bool MyOptimization::readLaserData(std::string filename)
{
  std::ifstream fileIn;
  bool ok = false;
  fileIn.open(filename);
  if(fileIn.is_open())
    ok = true;
  double week,tow,x,y,z,qx,qy,qz,qw;
  while (fileIn >> week >> tow >> x >> y >> z >> qx >> qy >> qz >> qw)
  {
      Eigen::Quaterniond quat(qw,qx,qy,qz);
      Eigen::Vector3d vect(x,y,z);
      Eigen::Affine3d af;
      af.linear() = quat.toRotationMatrix();
      af.translation() = vect;
      LaserPose laserPose(af, week, tow);
      laserPoses.push_back(laserPose);
  }
  fileIn.close();
  std::cout << "Read " << laserPoses.size() << " poses" << std::endl;
  return ok;
  
}

void MyOptimization::addLaserEdge(int week, double tow)
{
  // Check if previous poses exists
  if (optimizationResults.size() < 1)
    return;

  // Check if there is missing GPS vertex:
  // if (optimizationResults.back())

  // Previous GPS position
  OptimizationResults prevGPSPos = optimizationResults.back();
  // Actual GPS Time of Week
  double gpsTow = tow;
  // First Laser position
  double firstLaserTow = laserPoses[0].getTow();
   // Wait for next GPS pos
  if (firstLaserTow > gpsTow)
    return;

  static int lastLaserIdx = 1;
  int matchedLaserPose = -1;
  for (int i=lastLaserIdx; i < laserPoses.size(); i++)
  {
    double laserTow = laserPoses[i].getTow();

    if (laserTow < gpsTow)
      continue;
    else if (laserTow >= gpsTow)
    {
      double prevTimeDiff = laserPoses[i-1].getTow() - gpsTow;
      double actTimeDiff = laserTow - gpsTow;
      // Should be smaller/equal 0
      if (prevTimeDiff > 0)
        return;

      if(abs(prevTimeDiff) < actTimeDiff)
        matchedLaserPose = i-1;
      else
        matchedLaserPose = i;
      // lastLaserIdx = matchedLaserPose;
      break;
    }
  }
  // Return if no match found or its the first one
  if (matchedLaserPose == -1 || matchedLaserPose == 0)
    return;

  std::cout << "Matched laser pose:  " << matchedLaserPose << std::endl; 

  Eigen::Affine3d prevLaserPose =  laserPoses[lastLaserIdx].getPose();
  Eigen::Affine3d actLaserPose =  laserPoses[matchedLaserPose].getPose();
  
  lastLaserIdx = matchedLaserPose;

  // Calculate pose increment
  // ToDo: what coordinate frame?
  // ToDo: estimate direction vector based on GPS speed
  Eigen::Affine3d delta = prevLaserPose.inverse() * actLaserPose;
  
  // Set previous vertex as fixed, so its not optimized
  optimizer.vertex(prevGPSPos.getRoverVertexId())->setFixed(true);

  // // Add SE3 edge between GPS vertices
  // g2o::EdgeSE3 *edgeLaser = new g2o::EdgeSE3;

  // g2o::Isometry3 estimate = g2o::Isometry3::Identity();
  // Eigen::Matrix4d tmp = prevGPSPos.getEstimatedRoverPose() * delta.matrix();
  // estimate.matrix() = tmp;
  // // estimate.translation() = est.translation()*0.5;
  // g2o::VertexSE3 *ve3 = dynamic_cast<g2o::VertexSE3 *> (optimizer.vertex(lastVertexId - numBiases));
  // ve3->setEstimate(estimate);

  // edgeLaser->setVertex(0, dynamic_cast<g2o::VertexSE3 *>(optimizer.vertex(prevGPSPos.getRoverVertexId())));
  // edgeLaser->setVertex(1, dynamic_cast<g2o::VertexSE3 *>(optimizer.vertex(lastVertexId - numBiases)));

  // // Add measurement
  // g2o::Isometry3 measurement = g2o::Isometry3::Identity();
  // measurement = delta.matrix();
  // edgeLaser->setMeasurement(measurement);
  // // Add information matrix
  // g2o::MatrixN<6> information = g2o::MatrixN<6>::Identity();
  // edgeLaser->setInformation(information);
  // edgeLaser->setLevel(optLevel);
  // // Add edge to optimization
  // optimizer.addEdge(edgeLaser); 

  // Add DistanceEdge between GPS poses
  g2o::DistanceEdge *edgeDistance = new g2o::DistanceEdge;

  edgeDistance->setVertex(0, dynamic_cast<g2o::VertexSE3 *>(optimizer.vertex(prevGPSPos.getRoverVertexId())));
  edgeDistance->setVertex(1, dynamic_cast<g2o::VertexSE3 *>(optimizer.vertex(lastVertexId - numBiases)));

  // Add measurement
  double distance = delta.translation().norm();
  edgeDistance->setMeasurement(distance);
  // Add information matrix
  double  information = 1.0;
  edgeDistance->setInformation(information);
  edgeDistance->setLevel(optLevel);
  // Add edge to optimization
  optimizer.addEdge(edgeDistance); 
}

void addLaserVertex()
{
    // g2o::VertexSE3 *vertex = new g2o::VertexSE3;
    // g2o::Isometry3 estIso = g2o::Isometry3::Identity();
    // estIso.translation() = est;
    // vertex->setEstimate(estIso);
    // vertex->setId(++lastVertexId);
    // vertex->setFixed(false);
    // optimizer.addVertex(vertex);
}