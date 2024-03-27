/*
Generic solver class
No outlier removal in this class
author: Yun Chang, Luca Carlone
*/

#include "KimeraRPGO/GenericSolver.h"

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/PriorFactor.h>

#include <vector>

namespace KimeraRPGO {

// 构造函数
GenericSolver::GenericSolver(Solver solvertype,
                             std::vector<char> special_symbols)
    : values_(gtsam::Values()),  // gtsam::Values()是一个空的值容器
      nfg_(gtsam::NonlinearFactorGraph()),  // 非线性因子图 空的因子图
      solver_type_(solvertype),             // 求解器类型
      special_symbols_(special_symbols),    // 特殊符号
      debug_(true),                         // 调试标志
      log_(false) {}                        // 日志标志

// 常量成员函数 检查是否为特殊符号
bool GenericSolver::isSpecialSymbol(char symb) const {
  for (size_t i = 0; i < special_symbols_.size(); i++) {
    if (special_symbols_[i] == symb) return true;
  }
  return false;
}

void GenericSolver::updateValues(const gtsam::Values& values) {
  // 遍历values
  // v.key表示键 v.value表示值
  for (const auto& v : values) {
    if (values_.exists(v.key)) {  // 如果values_中存在v.key相同的value
      values_.update(v.key, v.value);  // 则更新v.key和v.value
    } else if (temp_values_.exists(
                   v.key)) {  // 如果temp_values_中存在v.key相同的value
      temp_values_.update(v.key,
                          v.value);  // 则更新temp_values_根据v.key和v.value
    }
  }
}

bool GenericSolver::addAndCheckIfOptimize(
    const gtsam::NonlinearFactorGraph& nfg,
    const gtsam::Values& values) {
  // add new values and factors
  nfg_.add(nfg);           // 将新的非线性因子图添加到nfg_中
  values_.insert(values);  // 将新的值插入到values_中

  // Do not optimize for just odometry (between) additions
  // 如果nfg.size() == 1 && nfg[0]->keys().size() == 2 && values.size() ==
  // 1则不优化
  if (nfg.size() == 1 && nfg[0]->keys().size() == 2 && values.size() == 1) {
    return false;
  }

  // nothing added so no optimization
  if (nfg.size() == 0 && values.size() == 0) {
    return false;
  }

  return true;
}

// 更新函数
void GenericSolver::update(const gtsam::NonlinearFactorGraph& nfg,
                           const gtsam::Values& values,
                           const gtsam::FactorIndices& factorsToRemove) {
  // TODO(Yun) Do we have unittests for generic (no outlier-rejection) update?
  // remove factors
  bool remove_factors = false;       // 是否删除因子
  if (factorsToRemove.size() > 0) {  // 如果要删除的因子数量大于0
    remove_factors = true;
  }
  for (size_t index : factorsToRemove) {  // 遍历要删除的因子
    nfg_[index].reset();                  // 直接将因子置空
  }

  bool process_lc =
      addAndCheckIfOptimize(nfg, values);  // 添加新的因子和值并检查是否优化

  if (process_lc || remove_factors) {  // 如果需要处理lc或者删除因子
    // optimize
    gtsam::Values result;                         // 优化结果
    gtsam::Values full_values = values_;          // 完整的值
    gtsam::NonlinearFactorGraph full_nfg = nfg_;  // 完整的非线性因子图
    full_values.insert(temp_values_);  // 将temp_values_插入到full_values中
    full_nfg.add(temp_nfg_);           // 将temp_nfg_添加到full_nfg中
    if (solver_type_ == Solver::LM) {  // 如果求解器类型为LM
      gtsam::LevenbergMarquardtParams params;  // LM参数
      if (debug_) {
        params.setVerbosityLM("SUMMARY");
        log<INFO>("Running LM");
      }
      // TODO: check if diagonalDamping is a good idea and the mean
      params.diagonalDamping = true;  //  use diagonal of Hessian
      result = gtsam::LevenbergMarquardtOptimizer(full_nfg, full_values, params)
                   .optimize();               // 优化结果
    } else if (solver_type_ == Solver::GN) {  // 如果求解器类型为GN
      gtsam::GaussNewtonParams params;
      if (debug_) {
        params.setVerbosity("ERROR");
        log<INFO>("Running GN");
      }
      result =  // 优化结果
          gtsam::GaussNewtonOptimizer(full_nfg, full_values, params).optimize();
    } else {
      log<WARNING>("Unsupported Solver");
      exit(EXIT_FAILURE);
    }
    updateValues(result);  // 根据优化结果更新值
  }
}

void GenericSolver::removeFactorsNoUpdate(
    gtsam::FactorIndices factorsToRemove) {
  // remove factors 并且不更新
  for (size_t index : factorsToRemove) {
    nfg_[index].reset();
  }
}

EdgePtr GenericSolver::removeLastFactor() {
  size_t num_factors = nfg_.size();  // 非线性因子图规模
  // 要删除的边为非线性因子图的最后一条边
  Edge removed_edge =
      Edge(nfg_[num_factors - 1]->front(), nfg_[num_factors - 1]->back());
  // 删除非线性因子图的最后一条边
  // std::prev()返回指向指定迭代器之前的迭代器 也就是end()-1
  // 针对std::vector std::string std::deque std::list
  // std::forward_list等序列容器 erase()返回的是删除元素的下一个元素的迭代器
  // 如果删除的是最后一个元素则返回end()
  // std::set std::multiset std::map std::multimap等关联容器 erase()返回void
  // C++17中引入的std::unordered_set std::unordered_multiset std::unordered_map
  // std::unordered_multimap等无序容器 erase返回被删除元素的数量
  nfg_.erase(std::prev(nfg_.end()));
  // 返回一个独占所有权的对象指针
  // 和传统裸指针相比，unique_ptr具有更强的所有权语义 开销更小 不能拷贝
  // unique_ptr是一个独占所有权的智能指针，它允许多个指针指向同一个对象，但是只有一个unique_ptr指针拥有它
  // 其拷贝构造函数和赋值运算符被删除，因此unique_ptr不能被拷贝
  // 只能用std::move()转移所有权
  return make_unique<Edge>(removed_edge);
}

void GenericSolver::removePriorsWithPrefix(const char& prefix) {
  // First make copy of nfg_ 复制一份非线性因子图
  const gtsam::NonlinearFactorGraph nfg_copy = nfg_;
  // Clear nfg_
  nfg_ = gtsam::NonlinearFactorGraph();
  // Iterate and pick out non prior factors and prior factors without key with
  // prefix
  for (auto factor : nfg_copy) {
    auto prior_factor_3d =
        factor_pointer_cast<gtsam::PriorFactor<gtsam::Pose3>>(factor);
    if (prior_factor_3d) {
      gtsam::Symbol node(prior_factor_3d->key());
      if (node.chr() != prefix) {
        nfg_.add(factor);
      }
      continue;
    }
    // Check for 2D prior factors
    auto prior_factor_2d =
        factor_pointer_cast<gtsam::PriorFactor<gtsam::Pose2>>(factor);
    if (prior_factor_2d) {
      gtsam::Symbol node(prior_factor_2d->key());
      if (node.chr() != prefix) {
        nfg_.add(factor);
      }
      continue;
    }

    nfg_.add(factor);
  }
}

}  // namespace KimeraRPGO
