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

#include "../include/raw_gnss_rtklib/my_g2o.h"
#include "../include/raw_gnss_rtklib/GPSEdge.h"

int my_g2o_main(int iter, const obsd_t *obs, int n, const double *rs,
                   const double *dts, const double *vare, const int *svh,
                   const nav_t *nav, const double *x, const prcopt_t *opt,
                   double *v1, double *H, double *var, double *azel, int *vsat,
                   double *resp, int *ns)
{

  
  for (int i =0; i < 10; i++)
  {
   
    std::cout << std::setprecision(16) << "x: " << rs[i*6] << " y: " << rs[1+i*6] << " z: " << rs[2+i*6] << "  ";
    std::cout << "dts: " << dts[i*2]*1E9 << " ";
    std::cout << "Var: " << var[i] << " ";
    std::cout << std::endl;
  }
  // Necessary data
  // Satellite position, dts, var, pseudorange (L1, L2/ combined), 

  int numPoints = 100;
  int maxIterations = 100;
  bool verbose = true;
  
  // Optimizer
  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(false);
 
  // allocate the solver
  g2o::OptimizationAlgorithmProperty solverProperty;
  optimizer.setAlgorithm(g2o::OptimizationAlgorithmFactory::instance()->construct("gn_dense", solverProperty));
  int id = 0;
  std::vector<g2o::VertexSE3*> vertices;

  // Add vertex to be found
  g2o::VertexSE3 *v = new g2o::VertexSE3;
  g2o::Isometry3 tf = g2o::Isometry3::Identity();
  tf.translation() = Eigen::Vector3d(1,1,1);
  v->setEstimate(tf);
  v->setId(id++);
  v->setFixed(false);
  vertices.push_back(v);
  // optimizer.addVertex(v);
  double pos[] = {0,0,1,  1, 0,0,  0,1,0, 0,0,-1};
  for (int i = 0; i < 4; i++)
  {
    g2o::VertexSE3* v = new g2o::VertexSE3;
    v->setId(id++);
    Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
    Eigen::Vector3d trans = Eigen::Vector3d(pos[i*3],pos[i*3 +1], pos[i*3 +2]);
    g2o::Isometry3 tf = g2o::Isometry3();
    tf = rot;
    tf.translation() = trans;
    // std::cout << "TF matrix: " << tf.matrix() << std::endl;
    v->setEstimate(tf);
    v->setFixed(true);
    vertices.push_back(v);
    // optimizer.addVertex(v);
  }

  Eigen::Matrix<double,6,6> information = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix3d transNoise =  Eigen::Matrix3d::Zero();
  Eigen::Matrix3d rotNoise =  Eigen::Matrix3d::Zero();
  for (int i =0; i <3 ; i++)
  {
    double transNoiseVal = 0.5;
    double rotNoiseVal = 0.1;
    // Translation noise
    transNoise(i,i) = std::pow(transNoiseVal, 2);
    // Rotation noise
    rotNoise(i, i) = std::pow(rotNoiseVal, 2);
  }
  information.block<3,3>(0,0) = transNoise.inverse();
  information.block<3,3>(3,3) = rotNoise.inverse();

  // std::cout << information << std::endl;
  // SE3 Edges
  // std::vector<g2o::VertexSE3 *> edges;
  // for (int i = 1; i < vertices.size(); i++)
  // {
  //   g2o::VertexSE3* prev = vertices[i-1];
  //   g2o::VertexSE3 *cur = vertices[i];
    // g2o::EdgeSE3 *e = new g2o::EdgeSE3;
  //   e->setVertex(0, prev);
  //   e->setVertex(1, cur);
  //   e->setInformation(information);
  //   // e->setMeasurement();
  //   // edges.push_back(e);
  // }

double distances[] = {1,1,1,1};
  std::vector<g2o::GPSEdge *> edges;
  for (int i = 1; i < vertices.size(); i++)
  {
    g2o::VertexSE3* next = vertices[i];
    g2o::GPSEdge *e = new  g2o::GPSEdge;
    e->setVertex(0,vertices[0]);
    e->setVertex(1, next);
    e->setInformation(1/0.5);
    e->setMeasurement(distances[i-1]);
    edges.push_back(e);
    // optimizer.addEdge(e);
  }

  //  optimizer.initializeOptimization();
  // optimizer.setVerbose(verbose);
  // optimizer.optimize(maxIterations);

  using namespace g2o;
  using namespace std;
   ofstream fileOutputStream;
  string outFilename = "-";
  if (outFilename != "-") {
    cerr << "Writing into " << outFilename << endl;
    fileOutputStream.open(outFilename.c_str());
  } else {
    cerr << "writing to stdout" << endl;
  }

  std::string vertexTag = g2o::Factory::instance()->tag(vertices[0]);
  std::string edgeTag = g2o::Factory::instance()->tag(edges[0]);

  ostream& fout = outFilename != "-" ? fileOutputStream : std::cout;
  for (size_t i = 0; i < vertices.size(); ++i) {
    VertexSE3* v = vertices[i];
    fout << vertexTag << " " << v->id() << " ";
    v->write(fout);
    fout << endl;
    if (i > 0)
      fout << "FIX " << v->id() << endl;
  }

  for (size_t i = 0; i < edges.size(); ++i) {
     g2o::GPSEdge* e = edges[i];
    VertexSE3* from = static_cast<VertexSE3*>(e->vertex(0));
    VertexSE3* to = static_cast<VertexSE3*>(e->vertex(1));
    fout << edgeTag << " " << from->id() << " " << to->id() << " ";
    e->write(fout);
    fout << endl;
  }

  // optimizer.addVertex(v);
  optimizer.load("gps.g2o");
  // optimizer.save("gps4.g2o");
  optimizer.initializeOptimization();
  optimizer.setVerbose(verbose);
  optimizer.optimize(maxIterations);
  // optimizer.ve
  // optimizer.vertices[0]
  ofstream fileOut;
fileOut.open("g2o_result.txt");
  for (auto it = optimizer.vertices().begin(); it != optimizer.vertices().end(); ++it)
  {
    g2o::VertexSE3 *v = static_cast< g2o::VertexSE3 *>(it->second);
    std::cout << std::setprecision(12) << "ID: " << v->id() << endl << v->estimate().matrix() << std::endl;
    if (v->id() == 0)
    {
      Eigen::Matrix4d m_out= v->estimate().matrix();
      fileOut << std::setprecision(15) <<  m_out(0,3) << " " << m_out(1,3) << " " << m_out(2,3);
    }
  }
  if (verbose);


  return 0;
}