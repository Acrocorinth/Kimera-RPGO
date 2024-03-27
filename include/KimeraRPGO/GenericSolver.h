/*
Generic backend solver class
author: Yun Chang, Luca Carlone
*/

#pragma once

#include <string>
#include <vector>

// enables correct operations of GTSAM (correct Jacobians)
#define SLOW_BUT_CORRECT_BETWEENFACTOR
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "KimeraRPGO/Logger.h"
#include "KimeraRPGO/SolverParams.h"
#include "KimeraRPGO/utils/TypeUtils.h"

namespace KimeraRPGO {

// 常规求解器
class GenericSolver {
 public:
  // 构造函数
  GenericSolver(Solver solvertype = Solver::LM,
                std::vector<char> special_symbols = std::vector<char>());
  // solvertype = 1 for LevenbergMarquardt, 2 for GaussNewton
  // special symbols denote non odometry factors - perhaps semantics

  // 将析构函数定义为虚函数，以便在派生类中进行覆盖，以便在删除基类指针时调用派生类的析构函数
  // 为了防止内存泄漏，通常将基类的析构函数定义为虚函数,实现多态
  virtual ~GenericSolver() = default;

  // update the graph with new factors and values
  void update(
      const gtsam::NonlinearFactorGraph& nfg = gtsam::NonlinearFactorGraph(),
      const gtsam::Values& values = gtsam::Values(),
      const gtsam::FactorIndices& factorsToRemove = gtsam::FactorIndices());

  // remove factors from the graph
  void removeFactorsNoUpdate(
      gtsam::FactorIndices factorsToRemove = gtsam::FactorIndices());

  // size of the graph
  size_t size() { return nfg_.size(); }

  // 成员函数被定义为内联函数的目的是为了提高代码的执行效率,内联函数的定义在编译时会被直接插入到调用它的地方
  // 而不是通过函数调用的方式进行调用 可以减少函数调用的开销 提高程序的运行速度
  // 以下成员函数比较简单，适合定义为内联函数
  inline gtsam::Values calculateEstimate() const {
    return values_;
  }  // 返回优化后的值
  inline gtsam::Values calculateBestEstimate() const { return values_; }
  inline gtsam::Values getLinearizationPoint() const { return values_; }
  inline gtsam::NonlinearFactorGraph getFactorsUnsafe() const { return nfg_; }

  inline gtsam::Values getTempValues() const { return temp_values_; }
  inline gtsam::NonlinearFactorGraph getTempFactorsUnsafe() const {
    return temp_nfg_;
  }
  inline void updateTempFactorsValues(
      const gtsam::NonlinearFactorGraph& temp_nfg,
      const gtsam::Values& temp_values) {
    temp_nfg_.add(temp_nfg);
    temp_values_.insert(temp_values);
  }
  inline void replaceTempFactorsValues(
      const gtsam::NonlinearFactorGraph& temp_nfg,
      const gtsam::Values& temp_values) {
    temp_nfg_ = temp_nfg;
    temp_values_ = temp_values;
  }
  inline void clearTempFactorsValues() {
    temp_nfg_ = gtsam::NonlinearFactorGraph();
    temp_values_ = gtsam::Values();
  }

  void print() const { values_.print(""); }

  // 设置调试标志 关闭debug
  void setQuiet() { debug_ = false; }

  // remove last added factor
  EdgePtr removeLastFactor();  // remove last added factor

  // remove last added factor with certain prefix
  // 移除最后添加的具有特定前缀的因子
  void removePriorsWithPrefix(const char& prefix);

  // 更新值
  void updateValues(const gtsam::Values& values);

  // 多态继承中 protected成员函数可以被派生类访问和调用
  // 但不能被外部类和对象访问，当基类的protected成员函数被声明为虚函数时,派生类可以通过覆盖该函数类实现自己的版本
  // 在多态继承中,通过基类指针或引用调用虚函数时
  // 会根据实际对象的类型来调用相应的函数 可以实现运行时的多态与动态绑定
  // 提供了更大的灵活性
 protected:
  // 添加并检查是否优化
  bool addAndCheckIfOptimize(
      const gtsam::NonlinearFactorGraph& nfg = gtsam::NonlinearFactorGraph(),
      const gtsam::Values& values = gtsam::Values());

 protected:
  // 函数末尾的const关键字表示该函数不会修改成员变量 属于常量成员函数
  // 除非成员变量被声明为mutable类型，否则常量成员函数不能修改成员变量
  // 常量成员函数可以调用非常量的成员函数
  // 但如果非常量成员函数修改了成员变量那么编译器会报错
  // mutable关键字可以用来修饰类的成员变量，被mutable修饰的变量将永远处于可变的状态，即使在一个const函数中
  // 也可以对其进行修改
  bool isSpecialSymbol(char symb) const;
  gtsam::Values values_;             // 优化后的值
  gtsam::NonlinearFactorGraph nfg_;  // 非线性因子图
  // Factors and values subjected to change
  gtsam::Values temp_values_;             // 临时值
  gtsam::NonlinearFactorGraph temp_nfg_;  // 临时非线性因子图

  Solver solver_type_;  // 求解器类型 LM代表LevenbergMarquardt GN代表GaussNewton
  std::vector<char> special_symbols_;  // 特殊符号
  bool debug_;                         // 调试标志
  bool log_;                           // 日志标志
  std::string log_folder_;             // 日志文件夹
};

}  // namespace KimeraRPGO