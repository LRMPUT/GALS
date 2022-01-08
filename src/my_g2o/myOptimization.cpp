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

    lastRoverPose = Eigen::Matrix4d::Identity();
    for (int i =0;i<numBiases;i++)
      lastBiasesValue[i] = 0;

    maxIterations = iter;
    firstPoseWithLaserEdgeId = -1;

}
void MyOptimization::addRoverVertex(const Eigen::Matrix4d &est)
{
    g2o::VertexSE3 *vertex = new g2o::VertexSE3;
    g2o::Isometry3 estIso = g2o::Isometry3::Identity();
    estIso.matrix() = est;
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

Eigen::Matrix4d MyOptimization::getLastRoverPose()
{
  return lastRoverPose;
}

std::array<double,5> MyOptimization::getLastBiasesValue()
{
  return lastBiasesValue;
}

void MyOptimization::optimize()
{
  optimizer.setVerbose(false);
  int windowSize = 50;
  int i;
  for (i = laserEdgesList.size() - 1; i > (laserEdgesList.size() - 1 - windowSize) && i > 1; i--)
  {
    laserEdgesList.at(i)->setLevel(optLevel);
  }

  
  // (optimizer.vertex(lastVertexId - numBiases))->setFixed(false);

  // for (int i = optimizationResults.size() - 1; i > (optimizationResults.size() - 1 - windowSize) && i > firstPoseWithLaserEdgeId +1; i--)
  // {
  //   optimizer.vertex(optimizationResults[i].getRoverVertexId())->setFixed(false);
  // }
  int fixedV = -1;
  if (optimizationResults.size() < windowSize)
  {
    if (optimizationResults.size() != 0)
    {
      (optimizer.vertex(optimizationResults[0].getRoverVertexId()))->setFixed(true);
      fixedV = 0;
    }
  }
  else
  {
    fixedV = optimizationResults.size() - windowSize;
     optimizer.vertex(optimizationResults[optimizationResults.size() - windowSize].getRoverVertexId())->setFixed(true);
  }

  std::cout << "LaserEdges used - from last to " << i <<   "  Fixed vertex : " << fixedV << std::endl;
  optimizer.initializeOptimization(optLevel++);
  optimizer.optimize(maxIterations);
  static int cnt = 0;
  // if(cnt++ % 200 == 0)
  //   {
  //     optimizer.setVerbose(true);
  //        std::cout << "Calling" << std::endl;
  //        for (int i = firstPoseWithLaserEdgeId + 1; i < optimizationResults.size(); i++)
  //         {
  //           g2o::VertexSE3 *v = static_cast<g2o::VertexSE3 *>(optimizer.vertex(optimizationResults[i].getRoverVertexId()));
  //           v->setFixed(false);
  //         }
  //        optimizer.initializeOptimization(99999);
  //        optimizer.optimize(10);
  //   }
  optimizer.setVerbose(false);
}

void MyOptimization::optimizeAll()
{
  optimizer.setVerbose(true);

  // Set all vertices as fixed
  for (auto it = optimizer.vertices().begin(); it != optimizer.vertices().end(); ++it)
  {
    g2o::VertexSE3 *v = static_cast<g2o::VertexSE3 *>(it->second);
   v->setFixed(true);
  }

  // firstPoseWithLaserEdgeId is fixed
  for (int i = firstPoseWithLaserEdgeId + 1; i < optimizationResults.size(); i++)
  {
    g2o::VertexSE3 *v = static_cast<g2o::VertexSE3 *>(optimizer.vertex(optimizationResults[i].getRoverVertexId()));
    v->setFixed(false);
    // g2o::Isometry3 est = g2o::Isometry3::Identity();
    // est.matrix() = optimizationResults[i].getEstimatedRoverPose();
    // g2o::Isometry3 est2 = v->estimate();
    // v->setEstimate(est);

  // if( i == firstPoseWithLaserEdgeId)
  // {
  //       std::cout << std::setprecision(15) << std::endl << "New:  " << std::endl  << est.matrix()  << "   vs "  << std::endl  << est2.matrix() << std::endl;

  //   g2o::VertexSE3 *v2 = static_cast<g2o::VertexSE3 *>(optimizer.vertex(optimizationResults[i-1].getRoverVertexId()));
  //   g2o::Isometry3 est2 = g2o::Isometry3::Identity();
  //   est2.matrix() = optimizationResults[i-1].getEstimatedRoverPose();
  //   g2o::Isometry3 est3 = v2->estimate();
  //   // v->setEstimate(est);

  //   std::cout << std::setprecision(15) << "Prev  "  << std::endl << est2.matrix()  << "   vs " << std::endl    << est3.matrix() << std::endl;
  // }
    // optimizer.vertex(optimizationResults[i].getRoverVertexId())->setFixed(false);
  }
    std::cout << "Starting whole optimization" << std::endl;
   // bool ok = optimizer.initializeOptimization(99999);
    // // optimizer.computeInitialGuess();
    // if(ok)
    //   std::cout << "Initalized" << std::endl;
    //  else
    //    std::cout << "Problem with initalization" << std::endl;

   // optimizer.optimize(5);
    std::cout << "Ended whole optimization" << std::endl;

    for (int j=0, i = firstPoseWithLaserEdgeId +1; i < optimizationResults.size(); i++,j++)
    {
      g2o::VertexSE3 *v = static_cast<g2o::VertexSE3 *>(optimizer.vertex(optimizationResults[i].getRoverVertexId()));
      // g2o::ParameterSE3Offset *p = static_cast<g2o::ParameterSE3Offset *>(optimizer.parameter(j));
      // std::cout << i << " " << p->offset().matrix() << std::endl;
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
  lastRoverPose = estPose;

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
  // Skip first pose, as it has no proper estimate etc
  if(firstPoseWithLaserEdgeId != -1)
    saveOutputToFile("g2o_sol.txt", estPose, week, tow);
}

void MyOptimization::saveOutputToFile(std::string filename, Eigen::Matrix4d pose, int week, double tow)
{
  std::ofstream fileOut;
  static bool initalizeFile = true;
  std::string folderPath = ros::package::getPath("raw_gnss_rtklib") + "/evaluation/g2o_output/";
  if (initalizeFile)
  {
    remove((folderPath + "g2o_sol.txt").c_str());
    remove((folderPath + "g2o_sol_all.txt").c_str());
    // fileOut.open(filename);
    // fileOut.close();
    initalizeFile = false;
  }
  fileOut.open(folderPath + filename, std::ios_base::app);
  Eigen::Quaterniond quat(pose.block<3,3>(0,0));
  fileOut << std::setprecision(15) << week << "," << tow << "," << pose(0, 3) << "," << pose(1, 3) << "," << pose(2, 3) << ","
                          << quat.x() << "," << quat.y() << "," << quat.z() << "," << quat.w() << std::endl;
  fileOut.close();
}

bool MyOptimization::readLaserData(std::string filename)
{
  double timeOffset = 3.0;
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
      LaserPose laserPose(af, week, tow + timeOffset);
      laserPoses.push_back(laserPose);
  }
  fileIn.close();
  std::cout << "Read " << laserPoses.size() << " poses" << std::endl;
  return ok;
  
}

void MyOptimization::addLaserEdge(int week, double tow, Eigen::Vector3d libPose)
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

  // First pose with laser edge
  if (firstPoseWithLaserEdgeId == -1)
    firstPoseWithLaserEdgeId = optimizationResults.size() - 1;

  std::cout << "Matched laser pose:  " << matchedLaserPose << std::endl; 

  Eigen::Affine3d prevLaserPose =  laserPoses[lastLaserIdx].getPose();
  Eigen::Affine3d prevPlus1LaserPose =  laserPoses[lastLaserIdx+1].getPose();
  Eigen::Affine3d actLaserPose =  laserPoses[matchedLaserPose].getPose();
  

  // Calculate pose increment
  // ToDo: what coordinate frame?
  // ToDo: estimate direction vector based on GPS speed

  // Calculate vector of laser translation to allign it with velocity
  Eigen::Vector3d laserVect = (prevLaserPose.inverse() * prevPlus1LaserPose).translation();
  std::array<double,3> vel = prevGPSPos.getVelocity();
  Eigen::Vector3d velVect = Eigen::Vector3d(vel[0], vel[1], vel[2]);
  static Eigen::Vector3d lastVelVect = Eigen::Vector3d(vel[0], vel[1], vel[2]);
  if(velVect.norm() < 1.0)
      velVect = lastVelVect;

  lastVelVect = velVect;
  Eigen::Vector3d laserVectNorm = laserVect.normalized(); 
  velVect.normalize();
  Eigen::Vector3d v = laserVectNorm.cross(velVect);
  double c = laserVectNorm.dot(velVect);
  double h = 1 / (1 + c);
  Eigen::Matrix3d mat;
  mat << 0, -v[2], v[1],
          v[2], 0, -v[0],
          -v[1], v[0], 0;
  Eigen::Matrix3d rot = Eigen::Matrix3d::Identity() + mat + (mat * mat * h);
  // Eigen::Vector3d alignedLaser = rot * laserVect;
  
  // Align first part of transform - if matchedLaserPose - lastLaserIdx == 1 then it's all   
  Eigen::Affine3d deltaGlobal = rot *prevLaserPose.inverse() * prevPlus1LaserPose;
  // However If (matchedLaserPose - lastLaserIdx > 1) then add delta:
  if (matchedLaserPose - lastLaserIdx > 1){
    deltaGlobal = deltaGlobal * prevPlus1LaserPose.inverse() * actLaserPose;
    std::cout << "Diff:  "  << matchedLaserPose - lastLaserIdx << "   Transform:   "   << deltaGlobal.translation().transpose() << std::endl;
  }
  lastLaserIdx = matchedLaserPose;

  // Eigen::Affine3d delta = prevLaserPose.inverse() * actLaserPose;
  
  // Set previous vertex as fixed, so its not optimized
  // optimizer.vertex(prevGPSPos.getRoverVertexId())->setFixed(true);

  // Add SE3 edge between GPS vertices
  g2o::EdgeSE3 *edgeLaser = new g2o::EdgeSE3();

  g2o::Isometry3 estimate = g2o::Isometry3::Identity();
  // Remove rotation
  // delta.matrix().block<3,3>(0,0) = Eigen::Matrix3d::Identity();
  // Eigen::Matrix4d tmp = delta.matrix() * prevGPSPos.getEstimatedRoverPose();
  Eigen::Affine3d deltaLocal = prevLaserPose.inverse() * actLaserPose;
  estimate.matrix() = prevGPSPos.getEstimatedRoverPose();
  estimate.translate(deltaGlobal.translation());
  // estimate.matrix().block<3,3>(0,0) = Eigen::Matrix3d::Identity();
  estimate.rotate(deltaLocal.linear());
  // estimate.translation() = estimate.translation()*2.5;
  g2o::VertexSE3 *ve3 = dynamic_cast<g2o::VertexSE3 *> (optimizer.vertex(lastVertexId - numBiases));
  //std::cout << ve3->estimate().matrix() <<  std::endl << prevGPSPos.getEstimatedRoverPose() << std::endl;
  estimate = prevGPSPos.getEstimatedRoverPose() * deltaLocal.matrix();
  ve3->setEstimate(estimate);

  edgeLaser->setVertex(0, dynamic_cast<g2o::VertexSE3 *>(optimizer.vertex(prevGPSPos.getRoverVertexId())));
  edgeLaser->setVertex(1, dynamic_cast<g2o::VertexSE3 *>(optimizer.vertex(lastVertexId - numBiases)));

  // Add measurement
  g2o::Isometry3 measurement = g2o::Isometry3::Identity();
  measurement = deltaLocal.matrix(); //prevGPSPos.getEstimatedRoverPose().inverse() * estimate.matrix();//delta.matrix();
  edgeLaser->setMeasurement(measurement);

  // Add information matrix
  g2o::MatrixN<6> information = 10.0 * g2o::MatrixN<6>::Identity();
  edgeLaser->setInformation(information);
  edgeLaser->setLevel(optLevel);
  // Add edge to optimization
  optimizer.addEdge(edgeLaser); 
  laserEdgesList.push_back(edgeLaser);


  // // Add DistanceEdge between GPS poses
  // g2o::DistanceEdge *edgeDistance = new g2o::DistanceEdge;

  // edgeDistance->setVertex(0, dynamic_cast<g2o::VertexSE3 *>(optimizer.vertex(prevGPSPos.getRoverVertexId())));
  // edgeDistance->setVertex(1, dynamic_cast<g2o::VertexSE3 *>(optimizer.vertex(lastVertexId - numBiases)));

  // // Add measurement
  // double distance = deltaLocal.translation().norm();
  // edgeDistance->setMeasurement(distance);
  // // Add information matrix
  // double  information2 = 1.0;
  // edgeDistance->setInformation(information2);
  // edgeDistance->setLevel(optLevel);
  // Add edge to optimization
  //optimizer.addEdge(edgeDistance); 


  // Edge from actual Position estimated by library
  // g2o::EdgeSE3XYZPrior *edgeXYZPrior = new g2o::EdgeSE3XYZPrior();
  // edgeXYZPrior->setVertex(0, static_cast<g2o::VertexSE3 *>(optimizer.vertex(lastVertexId - numBiases)));
  // // edgeXYZPrior->setVertex(0,ve3);
  // edgeXYZPrior->setMeasurement(libPose);
  // g2o::MatrixN<3> informationXYZ = 10.0 * g2o::MatrixN<3>::Identity();
  // edgeXYZPrior->setInformation(informationXYZ);
  // edgeXYZPrior->setLevel(optLevel);

  // g2o::ParameterSE3Offset *poseOffset = new g2o::ParameterSE3Offset;
  // static int id = 0;
  // poseOffset->setId(id);
  // optimizer.addParameter(poseOffset);

  // edgeXYZPrior->setParameterId(0,id++);
  // optimizer.addEdge(edgeXYZPrior); 

}

void MyOptimization::addVelToLastOptimResult(std::array <double,3> vel, double tow)
{
  OptimizationResults & last = optimizationResults.back();
  //Should be exactly the same
  if( abs(last.getTow() - tow) < 0.05)
    last.setVelocity(vel);
  else{
    // std::cout << std::setprecision(15) <<  "Tow: "  << last.getTow() << "   vs   " << tow << std::endl;
    // exit(0);
  }
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