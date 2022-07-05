#include "myOptimization.h"

MyOptimization::MyOptimization() : numBiases(NUMBIASES), lastVertexId(-1), optLevel(0), GPSEdgesListList(40000)
{
    optimizer.setVerbose(paramVerbose);

    g2o::SparseOptimizerTerminateAction* terminateAction = new g2o::SparseOptimizerTerminateAction;
    terminateAction->setGainThreshold(0.0001);
    terminateAction->setMaxIterations(paramMaxIterations);
    optimizer.addPostIterationAction(terminateAction);
    g2o::OptimizationAlgorithmProperty solverProperty;
    optimizer.setAlgorithm(g2o::OptimizationAlgorithmFactory::instance()->construct("lm_dense", solverProperty));

    lastRoverPose = Eigen::Matrix4d::Identity();
    for (int i = 0; i < numBiases; i++)
      lastBiasesValue[i] = 0;

    firstPoseWithLaserEdgeId = -1;

    actDopplVel[0] = actDopplVel[1] = actDopplVel[2] = 0;
    actDopplCov[0] = actDopplCov[1] = actDopplCov[2] = 0;
    actDopplTow = 0;

    std::cout << "Parameters: " << std::endl;
    std::cout << "paramVerbose: " << paramVerbose << std::endl;
    std::cout << "paramWindowSize: " << paramWindowSize << std::endl;
    std::cout << "paramMaxIterations: " << paramMaxIterations << std::endl;
    std::cout << "paramMaxIterationsEnd: " << paramMaxIterationsEnd << std::endl;
    std::cout << "paramOptimizeBiasesAgain: " << paramOptimizeBiasesAgain << std::endl;
    std::cout << "paramOptimizeBiasesAgainEnd: " << paramOptimizeBiasesAgainEnd << std::endl;
    std::cout << "paramLaserInform: " << paramLaserInform << std::endl;
    std::cout << "paramLaserTimeOffset: " << paramLaserTimeOffset << std::endl;
    std::cout << "paramFilterGPS: " << paramFilterGPS << std::endl;
    std::cout << "paramMaxGPSSpeed: " << paramMaxGPSSpeed << std::endl;
    std::cout << "paramMaxGPSVertSpeed: " << paramMaxGPSVertSpeed << std::endl;
    std::cout << "paramSkipGPSPoses: " << paramSkipGPSPoses << std::endl;    
    std::cout << "paramMaxAltToDstPct: " << paramMaxAltToDstPct << std::endl;
    std::cout << "paramDecimation: " << paramDecimation << std::endl;
    std::cout << "paramPosesToProcess: " << paramPosesToProcess << std::endl;
    std::cout << "paramDopplerInformFactor: " << paramDopplerInformFactor << std::endl;
    std::cout << "paramVelToDopplerRatiox10: " << paramVelToDopplerRatiox10 << std::endl;
    std::cout << "paramSlamOdometryPath: " << paramSlamOdometryPath << std::endl;
}
void MyOptimization::addRoverVertex(const Eigen::Matrix4d &est)
{
    g2o::VertexSE3 *vertex = new g2o::VertexSE3;
    g2o::Isometry3 estIso = g2o::Isometry3::Identity();
    estIso.matrix() = est;
    vertex->setEstimate(estIso);
    vertex->setId(++lastVertexId);
    vertex->setFixed(false);
    optimizer.addVertex(vertex);
}

void MyOptimization::addBiasDriftVertex()
{
  for (int i = 0; i < NUMDRIFTBIASES; i++)
  {
    g2o::BiasDriftVertex *vertex = new g2o::BiasDriftVertex;
    Eigen::Matrix<double, 1, 1> estMat = Eigen::Matrix<double, 1, 1>::Zero();
    vertex->setEstimate(estMat);
    vertex->setId(++lastVertexId);
    vertex->setFixed(false);
    optimizer.addVertex(vertex);
  }
}

void MyOptimization::addBiasesVertices(const std::array<double,NUMBIASES> &est)
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
    biasList.push_back(bv);
  }
}

void MyOptimization::addBiasDriftEdge(int week, double tow)
{
  static double prevTow = 0;
  if (prevTow != 0)
  {
    for (int i =0; i<NUMDRIFTBIASES; i++){
      g2o::BiasDriftEdge *biasDriftEdge = new g2o::BiasDriftEdge;
      OptimizationResults prevPose = optimizationResults.back();
      biasDriftEdge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(prevPose.getRoverVertexId() + 1 + i)));
      std::cout << "PrevBias ID: " << prevPose.getRoverVertexId() + 1 << " vs " << lastVertexId - numBiases * 2 << std::endl;
      if ((prevPose.getRoverVertexId() + 1) != (lastVertexId - numBiases * 2))
        std::cout << "PrevBias error: " << std::endl;
      biasDriftEdge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(lastVertexId - 4 + i)));
      biasDriftEdge->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(i))); // BiasDriftVertex

      double information = 100000.0;
      biasDriftEdge->setInformation(information);

      double measurement = tow - prevTow;
      Eigen::Matrix<double, 1, 1> measurementMat = Eigen::Matrix<double, 1, 1>::Zero();
      measurementMat[0] = measurement;
      biasDriftEdge->setMeasurement(measurementMat);

      biasDriftEdge->setLevel(optLevel);
      optimizer.addEdge(biasDriftEdge);

      int idx = (lastVertexId - NUMDRIFTBIASES /* Drift Biases Vertices*/) / (numBiases + 1);
      biasDriftEdgesList.push_back(biasDriftEdge);
    }
  }
  prevTow = tow;
}

void MyOptimization::addEdgeSatPrior(Eigen::Matrix<double, 4, 1> &measurement, double information, int sys)
{
  g2o::GPSEdgePrior *edgeSatPrior = new g2o::GPSEdgePrior;

  edgeSatPrior->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(lastVertexId-numBiases)));

  if      (sys == SYS_GPS){edgeSatPrior->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(lastVertexId-4)));}
  else if (sys==SYS_GLO) {edgeSatPrior->setVertex(1,  dynamic_cast<g2o::OptimizableGraph::Vertex *> (optimizer.vertex(lastVertexId-3)));}
  else if (sys==SYS_GAL) {edgeSatPrior->setVertex(1,  dynamic_cast<g2o::OptimizableGraph::Vertex *> (optimizer.vertex(lastVertexId-2)));}
  else if (sys==SYS_CMP) {edgeSatPrior->setVertex(1,  dynamic_cast<g2o::OptimizableGraph::Vertex *> (optimizer.vertex(lastVertexId-1)));}
  // Again GPS Vertex for QZS
  else if (sys==SYS_QZS) {edgeSatPrior->setVertex(1,  dynamic_cast<g2o::OptimizableGraph::Vertex *> (optimizer.vertex(lastVertexId-4)));}
  edgeSatPrior->setInformation(information);
  edgeSatPrior->setMeasurement(measurement);
  edgeSatPrior->setLevel(optLevel);
  optimizer.addEdge(edgeSatPrior);
  int idx = (lastVertexId - NUMDRIFTBIASES) / (numBiases + 1) ;
  GPSEdgesListList.at(idx).push_back(edgeSatPrior);
}

Eigen::Matrix4d MyOptimization::getLastRoverPose()
{
  return lastRoverPose;
}

std::array<double,NUMBIASES> MyOptimization::getLastBiasesValue()
{
  return lastBiasesValue;
}

void MyOptimization::optimize()
{
  int windowSize = paramWindowSize;

  // Set all laser edges in window for optimization (except actual one, as it is added in addLaserEdge())
  for (int i = laserEdgesList.size() - 1; i > ((int) laserEdgesList.size() - 1 - windowSize) && i >= 0; i--)
    laserEdgesList.at(i)->setLevel(optLevel);

  // Set all GPS constraints in window for optimization (except actual ones, as they are added in addEdgeSatPrior() )
  for (int i = optimizationResults.size() - 1; i > ((int) optimizationResults.size() - 1 - windowSize) && i >= 0; i--)
    for (int j = 0; j < GPSEdgesListList[i].size(); j++)
      (GPSEdgesListList[i]).at(j)->setLevel(optLevel);

  optimizer.initializeOptimization(optLevel++);
  optimizer.optimize(paramMaxIterations);

  // Set biases as fixed after optimization - reduces computation time, but might give worse results
  if(paramOptimizeBiasesAgain == false)
    for (int i = 0; i < numBiases; i++)
      optimizer.vertex(lastVertexId - i)->setFixed(true);
}

void MyOptimization::saveG2OFile()
{
  std::ofstream fout;
  fout.open(ros::package::getPath("gals") + "/evaluation/g2o_output/optim.g2o");

  for (auto it = optimizer.vertices().begin(); it != optimizer.vertices().end(); ++it)
  {
    g2o::VertexPointXYZ *v = static_cast<g2o::VertexPointXYZ *>(it->second);
    std::string vertexTag = g2o::Factory::instance()->tag(v);
    fout << vertexTag << " " << v->id() << " ";
    v->write(fout);
    fout << std::endl;
    if (v->fixed())
      fout << "FIX " << v->id() << std::endl;
  }
  for (auto it = optimizer.edges().begin(); it != optimizer.edges().end(); ++it)
  {
    g2o::GPSEdgePrior *e = dynamic_cast<g2o::GPSEdgePrior *>(*it);
    std::string edgeTag = g2o::Factory::instance()->tag(e);
    g2o::VertexPointXYZ *from = static_cast<g2o::VertexPointXYZ *>(e->vertex(0));
    g2o::BiasVertex *to = static_cast<g2o::BiasVertex *>(e->vertex(1));
    fout << edgeTag << " " << from->id() << " " << to->id() << " ";
    e->write(fout);
    fout << std::endl;
  }
}

void MyOptimization::optimizeAll()
{
  optimizer.setVerbose(true);

  // Set poses and biases vertices for optimization
    std::cout << "0" << std::endl;

  if (paramOptimizeBiasesAgainEnd == true)
    for (auto it = optimizer.vertices().begin(); it != optimizer.vertices().end(); ++it)
      (dynamic_cast<g2o::OptimizableGraph::Vertex *>(it->second))->setFixed(false);
  else
    for (auto it = optimizer.vertices().begin(); it != optimizer.vertices().end(); ++it)
      (dynamic_cast<g2o::OptimizableGraph::Vertex *>(it->second))->setFixed(true);
  std::cout << "1" << std::endl;
  // Unfix poses for optimization (firstPoseWithLaserEdgeId is not fixed, as its orientation should be estimated)
  std::cout << "firstPoseWithLaser:  " << firstPoseWithLaserEdgeId << std::endl;
  if (firstPoseWithLaserEdgeId == -1)
    firstPoseWithLaserEdgeId = 0;
  for (int i = firstPoseWithLaserEdgeId; i < optimizationResults.size(); i++)
    optimizer.vertex(optimizationResults[i].getRoverVertexId())->setFixed(false);

  // Set all laser edges for optimization
  for (int i = 0; i < laserEdgesList.size(); i++)
    laserEdgesList.at(i)->setLevel(optLevel);
  std::cout << "2" << std::endl;

  // Set all bias drift edges for optimization
  std::cout << "BiasDrift Edges: " << biasDriftEdgesList.size() << std::endl;
  for (int i = 0; i < biasDriftEdgesList.size(); i++)
    biasDriftEdgesList.at(i)->setLevel(optLevel);

  // Set all Doppler edges for optimization
  std::cout << "Doppler Edges: " << dopplerEdgesList.size() << std::endl;
  for (int i = 0; i < dopplerEdgesList.size(); i++)
    dopplerEdgesList.at(i)->setLevel(optLevel);

  // GPSEdgeListList does not contain all indices filled - there might be "jumps", as index correspond to pose number
  for (int i = GPSEdgesListList.size() - 1; i>= 0; i--)
     for (int j=0; j < GPSEdgesListList[i].size(); j++)
      (GPSEdgesListList[i]).at(j)->setLevel(optLevel);
  std::cout << "3" << std::endl;

  // Count number of all vertices set for optimization
  int unfixedNum = 0;
  for (auto it = optimizer.vertices().begin(); it != optimizer.vertices().end(); ++it)
    if ((dynamic_cast<g2o::OptimizableGraph::Vertex *>(it->second))->fixed() == false)
      unfixedNum++;
  std::cout << "4" << std::endl;

  std::cout << "Vertices too optimize: " << unfixedNum << std::endl;

  std::cout << "Starting whole optimization" << std::endl;
  bool ok = optimizer.initializeOptimization(optLevel);
  if (ok)
    std::cout << "Initalized" << std::endl;
  else
    std::cout << "Problem with initalization" << std::endl;

  optimizer.optimize(paramMaxIterationsEnd);
  std::cout << "Ended whole optimization" << std::endl;

  for (int i = 0; i < optimizationResults.size(); i++)
  {
    g2o::VertexSE3 *v = static_cast<g2o::VertexSE3 *>(optimizer.vertex(optimizationResults[i].getRoverVertexId()));
    saveOutputToFile("g2o_sol_all.txt", v->estimate().matrix(), optimizationResults[i].getWeek(), optimizationResults[i].getTow());
    std::array<double, NUMBIASES> biasesToFile;
    for (int j = 1; j < 1 + numBiases; j++){
      g2o::BiasVertex *bv = static_cast<g2o::BiasVertex *>(optimizer.vertex(optimizationResults[i].getRoverVertexId() + j));
      biasesToFile[j - 1] = bv->estimate()[0];
    }
    saveBiasesToFile("g2o_biases_all.txt", biasesToFile, optimizationResults[i].getWeek(), optimizationResults[i].getTow());
  }
  std::cout << "5" << std::endl;
}

void MyOptimization::processOutput(int week, double tow)
{
  // Estimated pose
  g2o::VertexSE3 *v = static_cast<g2o::VertexSE3 *>(optimizer.vertex(lastVertexId - numBiases));
  Eigen::Matrix4d estPose = v->estimate().matrix();
  lastRoverPose = estPose;
  // Estimated biases
  for (int i = 1; i < 1 + numBiases; i++)
  {
    g2o::BiasVertex *bv = static_cast<g2o::BiasVertex *>(optimizer.vertex(lastVertexId - numBiases + i));
    lastBiasesValue[i-1] = bv->estimate()[0];
  }

  // Store estimated values in separate vector
  OptimizationResults optimResult = OptimizationResults(lastVertexId - numBiases, estPose, lastBiasesValue, week, tow);
  optimizationResults.push_back(optimResult);
  // Save results to file
  saveOutputToFile("g2o_sol.txt", estPose, week, tow);
  saveBiasesToFile("g2o_biases.txt", lastBiasesValue, week, tow);
}

void MyOptimization::saveOutputToFile(std::string filename, Eigen::Matrix4d pose, int week, double tow)
{
  std::ofstream fileOut;
  static bool initalizeFile = true;
  std::string folderPath = ros::package::getPath("gals") + "/evaluation/g2o_output/";
  if (initalizeFile)
  {
    remove((folderPath + "g2o_sol.txt").c_str());
    remove((folderPath + "g2o_sol_all.txt").c_str());
    initalizeFile = false;
  }
  fileOut.open(folderPath + filename, std::ios_base::app);
  Eigen::Quaterniond quat(pose.block<3,3>(0,0));
  fileOut << std::setprecision(15) << week << "," << tow << "," << pose(0, 3) * 1e2 << "," << pose(1, 3) * 1e2 << "," << pose(2, 3) * 1e2 << ","
                          << quat.x() << "," << quat.y() << "," << quat.z() << "," << quat.w() << std::endl;
  fileOut.close();
}

void MyOptimization::saveBiasesToFile(std::string filename, std::array<double,NUMBIASES> biases, int week, double tow)
{
  std::ofstream fileOut;
  static bool initalizeFile = true;
  std::string folderPath = ros::package::getPath("gals") + "/evaluation/g2o_output/";
  if (initalizeFile)
  {
    remove((folderPath + "g2o_biases.txt").c_str());
    remove((folderPath + "g2o_biases_all.txt").c_str());
    initalizeFile = false;
  }
  fileOut.open(folderPath + filename, std::ios_base::app);
  fileOut << std::setprecision(15) << week << "," << tow << ",";
  for (int i=0; i<numBiases - 1; i++)
    fileOut << biases[i] << ",";
  fileOut << biases[numBiases - 1] << std::endl;
  fileOut.close();
}

bool MyOptimization::readLaserData(std::string filename)
{
  double timeOffset = paramLaserTimeOffset;
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
      af.translation() = vect / 1e2;
      LaserPose laserPose(af, week, tow + timeOffset);
      laserPoses.push_back(laserPose);
  }
  fileIn.close();
  std::cout << "Read " << laserPoses.size() << " poses" << std::endl;
  return ok;

}

void MyOptimization::addDopplerEdge(int week, double tow)
{

  std::cout << "Doppler Edge func "  << std::endl;
  if (optimizationResults.size() == 0 )
    return;

  OptimizationResults & prevGPSPos = optimizationResults.back();
 
  double timeDiff = tow - prevGPSPos.getTow();
  // If more than 3s
  if (timeDiff > 3.0){
    return;
    std::cout << "Time diff " << std::endl;
  }

  if (actDopplVel[0] == 0 && actDopplVel[1] == 0 && actDopplVel[2] == 0){
   std::cout << "Actual vel 0" << std::endl;
    return;
  }

  if (prevGPSPos.getVelocity()[0] == 0 && prevGPSPos.getVelocity()[1] == 0 && prevGPSPos.getVelocity()[2] == 0)
  {
    std::cout << "Prev vel 0" << std::endl;
    return;
  }
  g2o::DopplerEdge *dopplerEdge = new g2o::DopplerEdge();

  optimizer.vertex(prevGPSPos.getRoverVertexId())->setFixed(true);
  dopplerEdge->setVertex(0, dynamic_cast<g2o::VertexSE3 *>(optimizer.vertex(prevGPSPos.getRoverVertexId())));
  dopplerEdge->setVertex(1, dynamic_cast<g2o::VertexSE3 *>(optimizer.vertex(lastVertexId - numBiases)));

  g2o::VertexSE3 *ve3 = dynamic_cast<g2o::VertexSE3 *> (optimizer.vertex(lastVertexId - numBiases));

  // Add measurement
  std::array<double,3> velMeasurement; velMeasurement[0] = (actDopplVel[0] + prevGPSPos.getVelocity()[0]) / 2.0; velMeasurement[1] = (actDopplVel[1] + prevGPSPos.getVelocity()[1]) / 2.0 ; velMeasurement[2] = (actDopplVel[2] + prevGPSPos.getVelocity()[2]) / 2.0;
  std::array<double,3> dstMeasurement = velMeasurement; dstMeasurement[0] *= timeDiff; dstMeasurement[1] *= timeDiff; dstMeasurement[2] *= timeDiff;
  std::cout << "Doppler dst: " << dstMeasurement[0]/1e2 << "  "  << dstMeasurement[1]/1e2 << "  "  << dstMeasurement[2]/1e2 << "  "  << std::endl;
  std::cout << "Timediff:  " << timeDiff << std::endl;
  dopplerEdge->setMeasurement(dstMeasurement);
  // // Add information matrix
  std::array<double,3> information;
  information[0] = paramDopplerInformFactor / ( 100.0 * actDopplCov[0] * timeDiff);  information[1] = paramDopplerInformFactor / ( 100.0 * actDopplCov[1] * timeDiff);  information[2] = paramDopplerInformFactor / (100 * actDopplCov[2] * timeDiff);
  dopplerEdge->setInformation(information);
  dopplerEdge->setLevel(optLevel);
  // // Add edge to optimization
  optimizer.addEdge(dopplerEdge);
  dopplerEdgesList.push_back(dopplerEdge);

  g2o::Isometry3 estimate = g2o::Isometry3::Identity();
  estimate = prevGPSPos.getEstimatedRoverPose();
  estimate(0,3) += dstMeasurement[0]/1e2; estimate(1,3) += dstMeasurement[1]/1e2; estimate(2,3) += dstMeasurement[2]/1e2;
}

void MyOptimization::addLaserEdge(int week, double tow, Eigen::Vector3d libPose)
{
  // Actual GPS Time of Week
  double gpsTow = tow;
  // First Laser position
  double firstLaserTow = laserPoses[0].getTow();
   // Wait for next GPS pos
  if (firstLaserTow > gpsTow)
    return;

  static int lastLaserIdx = 1;
  static bool hasFirstMatch = false;
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
  {
    std::cout << "Matched: "  << matchedLaserPose << std::endl;
    return;
  }
  // First Match
  if( hasFirstMatch == false){
    hasFirstMatch = true;
    lastLaserIdx = matchedLaserPose;
    std::cout << "FirstPose" << std::endl;
    return;
  }

  // Check if previous poses exists
  if (optimizationResults.size() < 1)
    return;

  // Previous GPS position
  OptimizationResults prevGPSPos = optimizationResults.back();

  // First pose with laser edge
  if (firstPoseWithLaserEdgeId == -1)
    firstPoseWithLaserEdgeId = optimizationResults.size() - 1;

  std::cout << "Matched laser pose:  " << matchedLaserPose << std::endl;

  Eigen::Affine3d prevLaserPose =  laserPoses[lastLaserIdx].getPose();
  Eigen::Affine3d prevPlus1LaserPose =  laserPoses[lastLaserIdx+1].getPose();
  Eigen::Affine3d actLaserPose =  laserPoses[matchedLaserPose].getPose();

  // Calculate pose increment
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

  // Align first part of transform - if matchedLaserPose - lastLaserIdx == 1 then it's all
  Eigen::Affine3d deltaGlobal = rot *prevLaserPose.inverse() * prevPlus1LaserPose;
  // However If (matchedLaserPose - lastLaserIdx > 1) then add delta:
  if (matchedLaserPose - lastLaserIdx > 1){
    std::cout << "Diff:  "  << matchedLaserPose - lastLaserIdx << "   Transform:   "   << (prevLaserPose.inverse() * actLaserPose).translation().transpose() << std::endl;
  }
  double posesDiff = matchedLaserPose - lastLaserIdx;
  lastLaserIdx = matchedLaserPose;

  // Set previous vertex as fixed, so its not optimized
  // optimizer.vertex(prevGPSPos.getRoverVertexId())->setFixed(true);

  // Add SE3 edge between GPS vertices
  g2o::EdgeSE3 *edgeLaser = new g2o::EdgeSE3();

  g2o::Isometry3 estimate = g2o::Isometry3::Identity();
  Eigen::Affine3d deltaLocal = prevLaserPose.inverse() * actLaserPose;
  g2o::VertexSE3 *ve3 = dynamic_cast<g2o::VertexSE3 *> (optimizer.vertex(lastVertexId - numBiases));
  estimate = prevGPSPos.getEstimatedRoverPose() * deltaLocal.matrix();
  ve3->setEstimate(estimate);

  edgeLaser->setVertex(0, dynamic_cast<g2o::VertexSE3 *>(optimizer.vertex(prevGPSPos.getRoverVertexId())));
  edgeLaser->setVertex(1, dynamic_cast<g2o::VertexSE3 *>(optimizer.vertex(lastVertexId - numBiases)));

  // Add measurement
  g2o::Isometry3 measurement = g2o::Isometry3::Identity();
  measurement = deltaLocal.matrix(); 
  edgeLaser->setMeasurement(measurement);

  // Add information matrix
  g2o::MatrixN<6> information = (double) paramLaserInform / posesDiff * g2o::MatrixN<6>::Identity();
  edgeLaser->setInformation(information);
  edgeLaser->setLevel(optLevel);
  // Add edge to optimization
  optimizer.addEdge(edgeLaser);
  laserEdgesList.push_back(edgeLaser);
}

void MyOptimization::filterGPS(Eigen::Vector3d libPose, double tow, int &stat)
{
  if (paramFilterGPS == false)
    return;

  static double prevTow = 0;
  static double prevAlt = 0;
  static Eigen::Vector3d prevLibPose;

  double maxSpeed = paramMaxGPSSpeed; // m/s
  double maxVertSpeed = paramMaxGPSVertSpeed;
  double maxAltToDst = (double) paramMaxAltToDstPct / 100; // Altittude change to distance ratio
  // Calculate GPS Distance
  static int rejected = 0;
  double dstDiff = (libPose - prevLibPose).norm();
  if (dstDiff > (tow - prevTow) * maxSpeed * 1e-2){
    if (prevLibPose.norm() != 0 && prevTow != 0){
      stat = 0;
      rejected++;
      return;
    }
  }

  double posLLA[3];
  double libpos[3];
  libpos[0] = libPose[0] * 1e2;
  libpos[1] = libPose[1] * 1e2;
  libpos[2] = libPose[2] * 1e2;
  ecef2pos(libpos, posLLA);
  posLLA[2] -= geoidh(libpos);

  double altDiff = abs(posLLA[2] - prevAlt);

  // Check vertical velocity and vertical to total distance ratio
  if (altDiff > (tow - prevTow) * maxVertSpeed  || altDiff / (dstDiff * 1e2) > maxAltToDst ){
    if (prevAlt != 0){
      stat = 0;
      rejected++;
      return;
    }
  }

  // Save previous values
  prevLibPose[0] = libPose[0];
  prevLibPose[1] = libPose[1];
  prevLibPose[2] = libPose[2];
  prevTow = tow;
  prevAlt = posLLA[2];
}

void MyOptimization::filterGPSVel(Eigen::Vector3d libPose, double tow, int &stat)
{
  if (optimizationResults.size() == 0 )
    return;
  
  OptimizationResults & prevGPSPos = optimizationResults.back();
  
  if (actDopplVel[0] == 0 && actDopplVel[1] == 0 && actDopplVel[2] == 0)
    return;

  if (prevGPSPos.getVelocity()[0] == 0 && prevGPSPos.getVelocity()[1] == 0 && prevGPSPos.getVelocity()[2] == 0)
    return;

  if (paramFilterGPS == false)
    return;

  static double prevTow = 0;
  static Eigen::Vector3d prevLibPose;

  double velToDopplerRatio = paramVelToDopplerRatiox10; // m/s
  // Calculate GPS Distance
  static int rejected = 0;
  double dstDiff = (libPose - prevLibPose).norm();

  std::array<double,3> velAvg; velAvg[0] = (actDopplVel[0] + prevGPSPos.getVelocity()[0]) / 2.0; velAvg[1] = (actDopplVel[1] + prevGPSPos.getVelocity()[1]) / 2.0 ; velAvg[2] = (actDopplVel[2] + prevGPSPos.getVelocity()[2]) / 2.0;
  double velAvgNorm = sqrt(velAvg[0] * velAvg[0] + velAvg[1] * velAvg[1] + velAvg[2] * velAvg[2]);
  double poseVel = dstDiff * 1e2 / (tow - prevTow);

  if ((poseVel / velAvgNorm) > (velToDopplerRatio / 10.0) || (poseVel / velAvgNorm) < (1 / velToDopplerRatio / 10.0))
  {
    std::cout << "Rejecting based on Doppler velocity" << std::endl;
    stat = 0;
  }

  // Save previous values
  prevLibPose[0] = libPose[0];
  prevLibPose[1] = libPose[1];
  prevLibPose[2] = libPose[2];
  prevTow = tow;
}


void MyOptimization::addVelToLastOptimResult(std::array <double,3> vel, std::array <double,3> cov, double tow)
{

  // Save previous velocity
  if (optimizationResults.size() > 0)
  {
    OptimizationResults &last = optimizationResults.back();
    //Should be exactly the same
    last.setVelocity(actDopplVel);
  }

  // Set actual velocity
  actDopplVel = vel;
  actDopplTow = tow;
  actDopplCov = cov;
}

void addLaserVertex()
{
}