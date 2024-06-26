#include "KimeraRPGO/Logger.h"

#include <gtsam/base/GenericValue.h>
#include <gtsam/base/Lie.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Value.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/types.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Values-inl.h>
#include <gtsam/slam/BetweenFactor.h>

#include <fstream>

#include "KimeraRPGO/utils/TypeUtils.h"

// using means that we can use the class/variable/function without having to
using gtsam::BetweenFactor;
using gtsam::GenericValue;
using gtsam::Matrix3;
using gtsam::Matrix6;
using gtsam::NonlinearFactorGraph;
using gtsam::Point2;
using gtsam::Point3;
using gtsam::Pose2;
using gtsam::Pose3;
using gtsam::SharedNoiseModel;
using gtsam::Values;

using std::cout;
using std::endl;
using std::fstream;
using std::invalid_argument;

namespace KimeraRPGO {

namespace log_impl {

/**
 * @brief Construct a new Formatted Log:: Formatted Log object
 * 冒号后面的是初始化列表，用于初始化成员变量
 * @param level
 * @param msg
 */
FormattedLog::FormattedLog(log_level_t level, const std::string& msg)
    : level_(level) {
  ss_.reset(new std::stringstream());
  *ss_ << msg;
}

FormattedLog::~FormattedLog() {
  switch (level_) {
    case WARNING:
      // \033[1;33m is the ANSI escape code for yellow
      std::cout << "\033[1;33m" << ss_->str() << "\033[0m" << std::endl;
      break;
    case INFO:
      // \033[32m is the ANSI escape code for green
      std::cout << "\033[32m" << ss_->str() << "\033[0m" << std::endl;
      break;
    default:
      // \033[32m is the ANSI escape code for green
      std::cout << ss_->str() << std::endl;
      break;
  }
}

}  // namespace log_impl

/**
 * @brief
 *
 * @param graph 非线性因子图
 * @param estimate 估计值
 * @param filename 文件名
 */
void writeG2o(const NonlinearFactorGraph& graph,
              const Values& estimate,
              const std::string& filename) {
  // open file to write
  fstream stream(filename.c_str(), fstream::out);

  // save 2D poses
  for (const auto& key_value : estimate) {
    // dynamic_cast 用于将基类的指针或引用转换为派生类的指针或引用 (向下转型)
    // p为nullptr则返回nullptr
    auto p = dynamic_cast<const GenericValue<Pose2>*>(&key_value.value);
    if (!p) continue;
    const Pose2& pose = p->value();
    stream << "VERTEX_SE2 " << key_value.key << " " << pose.x() << " "
           << pose.y() << " " << pose.theta() << endl;
  }

  // save 3D poses
  for (const auto& key_value : estimate) {
    auto p = dynamic_cast<const GenericValue<Pose3>*>(&key_value.value);
    if (!p) continue;
    const Pose3& pose = p->value();
    const Point3 t = pose.translation();            // 平移
    const auto q = pose.rotation().toQuaternion();  // 旋转 四元数
    stream << "VERTEX_SE3:QUAT " << key_value.key << " " << t.x() << " "
           << t.y() << " " << t.z() << " " << q.x() << " " << q.y() << " "
           << q.z() << " " << q.w() << endl;
  }

  // save 2D landmarks 保存2D landmarks(特征点 路标点)
  for (const auto& key_value : estimate) {
    auto p = dynamic_cast<const GenericValue<Point2>*>(&key_value.value);
    if (!p) continue;
    const Point2& point = p->value();
    stream << "VERTEX_XY " << key_value.key << " " << point.x() << " "
           << point.y() << endl;
  }

  // save 3D landmarks
  for (const auto& key_value : estimate) {
    auto p = dynamic_cast<const GenericValue<Point3>*>(&key_value.value);
    if (!p) continue;
    const Point3& point = p->value();
    stream << "VERTEX_TRACKXYZ " << key_value.key << " " << point.x() << " "
           << point.y() << " " << point.z() << endl;
  }

  // save edges (2D or 3D)
  for (const auto& factor_ : graph) {
    // factor的含义是
    auto factor = factor_pointer_cast<BetweenFactor<Pose2>>(factor_);
    if (factor) {
      SharedNoiseModel model = factor->noiseModel();  // 噪声模型
      auto gaussianModel = factor_pointer_cast<gtsam::noiseModel::Gaussian>(
          model);  // 高斯噪声模型
      if (!gaussianModel) {
        model->print("model\n");
        throw invalid_argument("writeG2o: invalid noise model!");
      }
      /**
       * 信息矩阵 Info = R^T * R
         Hessian矩阵 是E相对于状态变量x的二阶偏导数矩阵
         H = J^T * W * J H是Hessian矩阵的近似矩阵
       在最大似然估计中，Hessian矩阵的逆矩阵就是信息矩阵
       在高斯分布的前提下,计算最大后验概率MAP时,协方差矩阵的逆矩阵就是信息矩阵
       *
       */
      Matrix3 Info = gaussianModel->R().transpose() * gaussianModel->R();
      Pose2 pose = factor->measured();  //.inverse(); 观测值
      stream << "EDGE_SE2 " << factor->key1() << " " << factor->key2() << " "
             << pose.x() << " " << pose.y() << " " << pose.theta();
      for (size_t i = 0; i < 3; i++) {
        for (size_t j = i; j < 3; j++) {
          stream << " " << Info(i, j);
        }
      }
      stream << endl;
    }

    auto factor3D = factor_pointer_cast<BetweenFactor<Pose3>>(factor_);
    if (factor3D) {
      SharedNoiseModel model = factor3D->noiseModel();
      auto gaussianModel =
          factor_pointer_cast<gtsam::noiseModel::Gaussian>(model);
      if (!gaussianModel) {
        model->print("model\n");
        throw invalid_argument("writeG2o: invalid noise model!");
      }
      Matrix6 Info = gaussianModel->R().transpose() * gaussianModel->R();
      const Pose3 pose3D = factor3D->measured();
      const Point3 p = pose3D.translation();            // 平移
      const auto q = pose3D.rotation().toQuaternion();  // 旋转 四元数形式
      stream << "EDGE_SE3:QUAT " << factor3D->key1() << " " << factor3D->key2()
             << " " << p.x() << " " << p.y() << " " << p.z() << " " << q.x()
             << " " << q.y() << " " << q.z() << " " << q.w();

      Matrix6 InfoG2o = Eigen::MatrixXd::Identity(6, 6);
      InfoG2o.block<3, 3>(0, 0) = Info.block<3, 3>(3, 3);  // cov translation
      InfoG2o.block<3, 3>(3, 3) = Info.block<3, 3>(0, 0);  // cov rotation
      InfoG2o.block<3, 3>(0, 3) = Info.block<3, 3>(0, 3);  // off diagonal
      InfoG2o.block<3, 3>(3, 0) = Info.block<3, 3>(3, 0);  // off diagonal

      for (size_t i = 0; i < 6; i++) {
        for (size_t j = i; j < 6; j++) {
          stream << " " << InfoG2o(i, j);
        }
      }
      stream << endl;
    }
  }
  stream.close();
}
}  // namespace KimeraRPGO
