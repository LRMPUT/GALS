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

MyOptimization::MyOptimization(bool verbose, int iter) : numBiases(5), lastBiasesValue(5), lastVertexId(-1)
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
    optimizer.setAlgorithm(g2o::OptimizationAlgorithmFactory::instance()->construct("lm_dense", solverProperty)); // GN - can start from (0,0),  LM - must have inital estimate

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
    vertex->setFixed(false);
    optimizer.addVertex(vertex);
}

void MyOptimization::addBiasesVertices(const std::vector<double> &est)
{
  if (est.size() != numBiases)
    std::cout << "Invalid size of bias vector";

  for (int i = 0; i < numBiases; i++)
  {
    g2o::BiasVertex *bv = new g2o::BiasVertex();
    Eigen::Matrix<double, 1, 1> estMat = Eigen::Matrix<double, 1, 1>::Zero();
    estMat[0] = est[i];
    bv->setId(++lastVertexId);
    bv->setFixed(false);
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
  // optimizer.addVertex(vertexSat);
  optimizer.addEdge(edgeSatPrior);
}

Eigen::Vector3d MyOptimization::getLastRoverPose()
{
  return lastRoverPose;
}

std::vector<double> MyOptimization::getLastBiasesValue()
{
  return lastBiasesValue;
}

void MyOptimization::optimize()
{
    optimizer.initializeOptimization();
    optimizer.optimize(maxIterations);
}

void MyOptimization::processOutput(int week, double tow)
{
                for (auto it = optimizer.vertices().begin(); it != optimizer.vertices().end(); ++it)
                {
                    g2o::VertexSE3 *v = static_cast<g2o::VertexSE3 *>(it->second);
                    if (v->id() == 0)
                    {
                        Eigen::Matrix4d m_out = v->estimate().matrix();
                        std::ofstream fileOut;
                        static bool initalizeFile = true;
                        if (initalizeFile)
                        {
                            fileOut.open("my_sol.txt");
                            fileOut.close();
                            initalizeFile = false;
                        }
                        fileOut.open("my_sol.txt", std::ios_base::app);
                        fileOut << std::setprecision(15) << week << "," << tow << "," << m_out(0, 3) << "," << m_out(1, 3) << "," << m_out(2, 3) << std::endl;
                        fileOut.close();
                        // if (initPose.translation() == Eigen::Vector3d(0,0,0))
                        static double lastTime = tow;
                        // initPose.translation() = Eigen::Vector3d(m_out(0, 3), m_out(1, 3), m_out(2, 3));
                        // for (int k = 0; k < biases.size(); k++)
                        {

                            // if (abs(lastBiases[k] - biases[k]->estimate()[0]) > 200)
                            // {
                            //     // showmsg("Big bias difference")
                            //     trace(2, "last bias: %13.3f   new bias: %13.3f time diff: %13.9f \n", lastBiases[k], biases[k]->estimate()[0], tow - lastTime);
                            // }
                            // lastBiases[k] = biases[k]->estimate()[0];
                        }
                            lastTime =  tow;
                            std::cout << std::setprecision(15) << "g2o out: " <<  week << " " << tow << " " <<  m_out(0, 3)<< " " <<  m_out(1, 3)<< " " <<  m_out(2, 3)<< std::endl;
                          lastRoverPose = Eigen::Vector3d( m_out(0, 3),  m_out(1, 3),  m_out(2, 3));
                        // showmsg("g2o out: %2d,%3.3f %15.5f %15.5f %15.5f", week, tow, m_out(0, 3), m_out(1, 3), m_out(2, 3));
                        // showmsg("bias : %15.5f  %15.5f  %15.5f  %15.5f %15.5f" ,biases[0]->estimate()[0], biases[1]->estimate()[0], biases[2]->estimate()[0],biases[3]->estimate()[0],biases[4]->estimate()[0]);
                    }
                    g2o::BiasVertex *bv = static_cast<g2o::BiasVertex *> (optimizer.vertex(2));
                    // if (v->id() == 1)
                    {
                      double bias1 = bv->estimate()[0];
                        std::cout << std::setprecision(15) << "bias out: " << bias1 << std::endl;
                    }
                }
}