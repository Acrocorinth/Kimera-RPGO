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

#include "KimeraRPGO/max_clique_finder/graphIO.h"

#include <map>
#include <string>
#include <vector>

namespace FMC {
CGraphIO::~CGraphIO() {
  m_vi_Vertices.clear();
  m_vi_OrderedVertices.clear();
  m_vi_Edges.clear();
  m_vd_Values.clear();
}

bool CGraphIO::readGraph(string s_InputFile, float connStrength) {
  // 获取文件扩展名
  string fileExtension = getFileExtension(s_InputFile);
  if (fileExtension == "mtx") {
    // matrix market format
    return ReadMatrixMarketAdjacencyGraph(s_InputFile, connStrength);
  } else if (fileExtension == "gr") {
    // gr format
    return ReadMeTiSAdjacencyGraph(s_InputFile);
  } else
    return false;
}

bool CGraphIO::ReadMatrixMarketAdjacencyGraph(string s_InputFile,
                                              float connStrength) {
  istringstream in2;
  string line = "";
  map<int, vector<int>> nodeList;
  map<int, vector<double>> valueList;
  int col = 0, row = 0, rowIndex = 0, colIndex = 0;
  int entry_counter = 0, num_of_entries = 0;
  double value;

  ifstream in(s_InputFile.c_str());
  if (!in) {
    cout << m_s_InputFile << " not Found!" << endl;
    return false;
  }

  char data[LINE_LENGTH];
  char banner[LINE_LENGTH];
  char mtx[LINE_LENGTH];
  char crd[LINE_LENGTH];
  char data_type[LINE_LENGTH];
  char storage_scheme[LINE_LENGTH];
  char* p;
  bool b_getValue = true;
  int num_upper_triangular = 0;

  // read the banner
  getline(in, line);
  strcpy(data, line.c_str());

  if (sscanf(data,
             "%s %s %s %s %s",
             banner,
             mtx,
             crd,
             data_type,
             storage_scheme) != 5) {
    cout << "Matrix file banner is missing!!!" << endl;
    return false;
  }

  // intersted about the forth part
  for (p = data_type; *p != '\0'; *p = tolower(*p), p++)
    ;

  if (strcmp(data_type, "pattern") == 0) b_getValue = false;

  getline(in, line);
  while (line.size() > 0 && line[0] == '%')  // ignore comment line
    getline(in, line);

  in2.str(line);
  in2 >> row >> col >> num_of_entries;

  if (row != col) {
    cout << "* WARNING: GraphInputOutput::ReadMatrixMarketAdjacencyGraph()"
         << endl;
    cout << "*\t row!=col. This is not a square matrix. Can't process." << endl;
    return false;
  }

  while (!in.eof() &&
         entry_counter <
             num_of_entries)  // there should be (num_of_entries+1) lines in the
                              // input file (excluding the comments)
  {
    getline(in, line);
    entry_counter++;

    if (line != "") {
      in2.clear();
      in2.str(line);

      in2 >> rowIndex >> colIndex >> value;
      rowIndex--;
      colIndex--;

      if (rowIndex < 0 || rowIndex >= row)
        cout << "Something wrong rowIndex " << rowIndex << " row " << row
             << endl;

      if (colIndex < 0 || colIndex >= col)
        cout << "Something wrong colIndex " << colIndex << " col " << col
             << endl;

      if (rowIndex == colIndex) {
        continue;
      }

      // This is to handle directed graphs. If the edge is already present,
      // skip. If not add.
      int exists = 0;
      for (size_t k = 0; k < nodeList[rowIndex].size(); k++) {
        if (colIndex == nodeList[rowIndex][k]) {
          exists = 1;
          break;
        }
      }

      if (exists == 1) {
        num_upper_triangular++;  //上三角矩阵中的元素个数
      } else {
        if (b_getValue) {
          if (value > connStrength) {
            nodeList[rowIndex].push_back(colIndex);
            nodeList[colIndex].push_back(rowIndex);
          }
        } else {
          nodeList[rowIndex].push_back(colIndex);
          nodeList[colIndex].push_back(rowIndex);
        }

        if (b_getValue && value > connStrength) {
          valueList[rowIndex].push_back(value);
          valueList[colIndex].push_back(value);
        }
      }
    }
  }

  // cout << "No. of upper triangular pruned: " << num_upper_triangular << endl;
  m_vi_Vertices.push_back(m_vi_Edges.size());

  for (int i = 0; i < row; i++) {
    m_vi_Edges.insert(m_vi_Edges.end(), nodeList[i].begin(), nodeList[i].end());
    m_vi_Vertices.push_back(m_vi_Edges.size());
  }

  if (b_getValue) {
    for (int i = 0; i < row; i++) {
      m_vd_Values.insert(
          m_vd_Values.end(), valueList[i].begin(), valueList[i].end());
    }
  }

  nodeList.clear();
  valueList.clear();
  CalculateVertexDegrees();
  return true;
}

bool CGraphIO::ReadEigenAdjacencyMatrix(Eigen::MatrixXd adjMatrix) {
  map<int, vector<int>> nodeList;  // 保存每个节点的邻接节点
  size_t col = 0, row = 0;         // 行数和列数

  // 行数和列数相等 均为节点数

  int num_upper_triangular = 0;  // 上三角矩阵中的元素个数

  row = adjMatrix.rows();
  col = adjMatrix.cols();

  for (size_t j = 0; j < col; j++) {  // 遍历每一列
    // 遍历完一列后,改列对应的节点的邻接节点已经全部找到,无需再遍历
    for (size_t i = j; i < row; i++) {  // 遍历每一行
      // 对角线元素和0元素不处理
      if (i == j || adjMatrix(i, j) == 0) {
        continue;
      }
      int exists = 0;
      // 遍历节点i的邻接节点 nodeList[i]表示获取键值为i的std::vrctor<int>的值
      for (size_t k = 0; k < nodeList[i].size(); k++) {
        // 遍历节点i的邻接节点,如果节点j已经存在,则不再添加
        if (j == nodeList[i][k]) {
          exists = 1;
          break;
        }
      }
      // 如果邻接节点不存在,则添加到节点i的邻接节点中
      if (exists == 1) {
        num_upper_triangular++;
      } else {
        nodeList[i].push_back(j);  //节点i的邻接节点添加j
        nodeList[j].push_back(i);  //节点j的邻接节点添加i
      }
    }
  }

  // 0号元素为m_vi_Edges.size()
  m_vi_Vertices.push_back(m_vi_Edges.size());

  for (int i = 0; i < row; i++) {  // r
    // 将每个节点的邻接节点添加到边数组中
    m_vi_Edges.insert(m_vi_Edges.end(), nodeList[i].begin(), nodeList[i].end());
    m_vi_Vertices.push_back(
        m_vi_Edges.size());  //每个节点的邻接节点在边数组中的起始位置
  }

  nodeList.clear();
  CalculateVertexDegrees();
  return true;
}

bool CGraphIO::ReadMeTiSAdjacencyGraph(string s_InputFile) { return true; }

// 计算每个顶点的入度
void CGraphIO::CalculateVertexDegrees() {
  // 顶点数
  int i_VertexCount = m_vi_Vertices.size() - 1;

  // 最大度和最小度初始化为-1
  m_i_MaximumVertexDegree = -1;
  m_i_MinimumVertexDegree = -1;

  // 遍历每个顶点
  for (int i = 0; i < i_VertexCount; i++) {
    // 第i个顶点的度等于下一个顶点的边的起始位置减去当前顶点的边的起始位置
    int i_VertexDegree = m_vi_Vertices[i + 1] - m_vi_Vertices[i];

    // 更新最大度和最小度
    if (m_i_MaximumVertexDegree < i_VertexDegree) {
      m_i_MaximumVertexDegree = i_VertexDegree;
    }

    if (m_i_MinimumVertexDegree == -1) {
      m_i_MinimumVertexDegree = i_VertexDegree;
    } else if (m_i_MinimumVertexDegree > i_VertexDegree) {
      m_i_MinimumVertexDegree = i_VertexDegree;
    }
  }

  // 计算平均度为边数除以顶点数
  m_d_AverageVertexDegree = (double)m_vi_Edges.size() / i_VertexCount;

  return;
}

string CGraphIO::getFileExtension(string fileName) {
  string::size_type result;
  string fileExtension = "";

  // 1. see if the fileName is given in full path
  /*result = fileName.rfind("/", fileName.size() - 1);
    if(result != string::npos)
    {
  //found the path (file prefix)
  //get the path, including the last DIR_SEPARATOR
  path = fileName.substr(0,result+1);
  //remove the path from the fileName
  fileName = fileName.substr(result+1);
  }
  */

  // 2. see if the fileName has file extension. For example ".mtx"
  result = fileName.rfind('.', fileName.size() - 1);
  if (result != string::npos) {
    // found the fileExtension
    // get the fileExtension excluding the '.'
    fileExtension = fileName.substr(result + 1);
    // remove the fileExtension from the fileName
    // fileName = fileName.substr(0,result);
  }

  // 3. get the name of the input file
  // name = fileName;

  return fileExtension;
}
}  // namespace FMC