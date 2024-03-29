/*   Description:  an I/O library for reading a graph



   Authors: Md. Mostofa Ali Patwary and Bharath Pattabiraman
            EECS Department, Northwestern University
            email: {mpatwary,bpa342}@eecs.northwestern.edu

   Copyright, 2014, Northwestern University
   See COPYRIGHT notice in top-level directory.

   Please site the following publication if you use this package:
   Bharath Pattabiraman, Md. Mostofa Ali Patwary, Assefaw H. Gebremedhin2,

   Wei-keng Liao, and Alok Choudhary.
   "Fast Algorithms for the Maximum Clique Problem on Massive Graphs with
   Applications to Overlapping Community Detection"

   http://arxiv.org/abs/1411.7460 */

#pragma once

#include <float.h>
#include <string.h>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#define LINE_LENGTH 256

using namespace std;

namespace FMC {
typedef std::vector<int> IntVector;

class CGraphIO {
 public:
  CGraphIO() {}
  virtual ~CGraphIO();

  // Read graph from file
  bool readGraph(string s_InputFile, float connStrength = -DBL_MAX);
  // 获取文件扩展名
  string getFileExtension(string fileName);
  // 从Matrix Market格式的文件中读取邻接图数据
  bool ReadMatrixMarketAdjacencyGraph(string s_InputFile,
                                      float connStrength = -DBL_MAX);
  // 从Eigen格式的文件中读取邻接图数据
  bool ReadEigenAdjacencyMatrix(Eigen::MatrixXd adjMatrix);
  // 从MeTiS格式的文件中读取邻接图数据
  bool ReadMeTiSAdjacencyGraph(string s_InputFile);
  // 计算图的顶点度
  void CalculateVertexDegrees();

  // 获取图的顶点数
  int GetVertexCount() { return m_vi_Vertices.size() - 1; }
  // 获取图的边数
  int GetEdgeCount() { return m_vi_Edges.size() / 2; }
  // 获取图的顶点最大度
  int GetMaximumVertexDegree() { return m_i_MaximumVertexDegree; }
  // 获取图的顶点最小度
  int GetMinimumVertexDegree() { return m_i_MinimumVertexDegree; }
  // 获取图的顶点平均度
  double GetAverageVertexDegree() { return m_d_AverageVertexDegree; }
  // 获取图的输入文件名
  string GetInputFile() { return m_s_InputFile; }

  //   // 获取图的顶点数组指针和边数组指针
  //   // 返回const指针，表示指向const对象的指针，不能通过该指针修改对象的值
  //   // const_integer含义是指向整型常量的指针，不能通过该指针修改整型常量的值
  //   const vector<int> *GetVerticesPtr() const { return &m_vi_Vertices; }
  //   const vector<int> *GetEdgesPtr() const { return &m_vi_Edges; }

  //   // 第一个函数不是常量成员函数,无法在一个常量对象上调用
  //   const vector<int> GetVertices() { return m_vi_Vertices; }
  //   // 第二个函数是常量成员函数，可以在一个常量对象上调用
  //   const vector<int> GetEdges() const { return m_vi_Edges; }
  vector<int>* GetVerticesPtr() { return &m_vi_Vertices; }
  vector<int>* GetEdgesPtr() { return &m_vi_Edges; }
  const vector<int> GetVertices() const { return m_vi_Vertices; }
  const vector<int> GetEdges() const { return m_vi_Edges; }

 public:
  int m_i_MaximumVertexDegree;
  int m_i_MinimumVertexDegree;
  double m_d_AverageVertexDegree;

  string m_s_InputFile;

  // 顶点数组，存储每个顶点的边在边数组中的起始位置(下标或者序号)
  vector<int> m_vi_Vertices;

  vector<int> m_vi_OrderedVertices;
  // 边数组，存储每个边的另一端的顶点序号
  vector<int> m_vi_Edges;
  vector<double> m_vd_Values;
};
}  // namespace FMC
