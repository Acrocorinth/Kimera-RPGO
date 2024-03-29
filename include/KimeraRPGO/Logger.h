/*
Simple logger class for prints and warnings
author: Yun Chang
*/

#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <sstream>

namespace KimeraRPGO {

// log levels
enum log_level_t {
  WARNING,
  INFO,
};

namespace log_impl {

class FormattedLog {
 public:
  // Constructor with log level and message
  FormattedLog(log_level_t level, const std::string& msg);

  // Destructor
  ~FormattedLog();

  // Operator overloading for << to append to the stringstream
  /**
   * @brief 将传入的值追加到字符串流中,
   * 并返回字符串流对象的引用，以便链式调用，即可以连续使用 << 运算符
   *
   * @tparam T
   * @param value
   * @return FormattedLog&
   */
  template <typename T>
  FormattedLog& operator<<(T value) {
    *ss_ << value;
    return *this;
  }

 protected:
  log_level_t level_;
  std::shared_ptr<std::stringstream> ss_;
};

}  // namespace log_impl

// Helper function. Class FormattedLog will not be used directly.
template <log_level_t level>
log_impl::FormattedLog log(const std::string& msg = "") {
  return log_impl::FormattedLog(level, msg);
}

void writeG2o(const gtsam::NonlinearFactorGraph& graph,
              const gtsam::Values& estimate,
              const std::string& filename);

}  // namespace KimeraRPGO
