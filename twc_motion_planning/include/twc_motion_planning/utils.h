#ifndef TWC_MOTION_PLANNING_UTILS_H
#define TWC_MOTION_PLANNING_UTILS_H

#include <tesseract_motion_planners/robot_config.h>
#include <tesseract_motion_planners/descartes/descartes_vertex_evaluator.h>
#include <descartes_light/interface/edge_evaluator.h>

namespace twc
{
enum class ProfileType
{
  ROBOT_ONLY,
  ROBOT_ON_RAIL,
  ROBOT_WITH_3AXIS_POSITIONER,
};

inline bool isRobotConfigValid(tesseract_planning::RobotConfig config)
{
  if (config != tesseract_planning::RobotConfig::NUT && config != tesseract_planning::RobotConfig::FUT)
    return false;

  return true;
}

inline bool isRobotConfigValid(tesseract_planning::RobotConfig start_config, tesseract_planning::RobotConfig end_config)
{
  if (start_config == end_config)
    return true;

  if (start_config == tesseract_planning::RobotConfig::NUT && end_config == tesseract_planning::RobotConfig::FUT)
    return true;

  if (start_config == tesseract_planning::RobotConfig::FUT && end_config == tesseract_planning::RobotConfig::NUT)
    return true;

  return false;
}

class DescartesStateValidator : public tesseract_planning::DescartesVertexEvaluator
{
public:
  DescartesStateValidator(Eigen::MatrixX2d limits,
                          tesseract_kinematics::ForwardKinematics::Ptr robot_only_kin)
    : limits_(std::move(limits)), robot_only_kin_(std::move(robot_only_kin))
  {
  }

  bool operator()(const Eigen::Ref<const Eigen::VectorXd>& vertex) const override
  {
    Eigen::Vector2i sign_correction(-1, 1);
    auto robot_config =
        tesseract_planning::getRobotConfig<double>(robot_only_kin_, vertex.tail(robot_only_kin_->numJoints()), sign_correction);

    if (!isRobotConfigValid(robot_config))
      return false;

    // Do not return redundant solutions
    Eigen::VectorXi start_redun = tesseract_planning::getJointTurns<double>(vertex.tail(3));
    if (start_redun != Eigen::Vector3i::Zero())
      return false;

    return tesseract_common::satisfiesPositionLimits(vertex, limits_);
  }

protected:
  Eigen::MatrixX2d limits_;
  tesseract_kinematics::ForwardKinematics::Ptr robot_only_kin_{ nullptr };
};

template <typename FloatType>
class RobotConfigEdgeEvaluator : public descartes_light::EdgeEvaluator<FloatType>
{
public:
  RobotConfigEdgeEvaluator(tesseract_kinematics::ForwardKinematics::ConstPtr robot_only_kin)
    : robot_only_kin_(robot_only_kin)
  {
  }

  std::pair<bool, FloatType> evaluate(const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>& start,
                                      const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>& end) const override
  {
    // Consider the edge:
    Eigen::Vector2i sign_correction(-1, 1);
    auto start_config = tesseract_planning::getRobotConfig<FloatType>(robot_only_kin_, start.tail(robot_only_kin_->numJoints()), sign_correction);
    auto end_config = tesseract_planning::getRobotConfig<FloatType>(robot_only_kin_, end.tail(robot_only_kin_->numJoints()), sign_correction);

    if (isRobotConfigValid(start_config, end_config))
      return std::make_pair(true, FloatType(0));

    return std::make_pair(false, FloatType(0));
  }

protected:
  tesseract_kinematics::ForwardKinematics::ConstPtr robot_only_kin_;
};

template <typename FloatType>
class WeightedEuclideanDistanceEdgeEvaluator : public descartes_light::EdgeEvaluator<FloatType>
{
public:
  WeightedEuclideanDistanceEdgeEvaluator(const Eigen::VectorXd& weights) : weights_(weights.cast<FloatType>()) {}

  std::pair<bool, FloatType> evaluate(const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>& start,
                                      const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>& end) const override
  {
    FloatType cost = (weights_.array() * (end - start).array().abs()).sum();
    return std::make_pair(true, cost);
  }

private:
  Eigen::Matrix<FloatType, Eigen::Dynamic, 1> weights_;
};

}

#endif // TWC_MOTION_PLANNING_UTILS_H
