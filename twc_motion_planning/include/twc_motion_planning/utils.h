#ifndef TWC_MOTION_PLANNING_UTILS_H
#define TWC_MOTION_PLANNING_UTILS_H
#include <tesseract_motion_planners/core/waypoint.h>
#include <tesseract_kinematics/core/validate.h>
#include <tesseract_motion_planners/descartes/descartes_motion_planner.h>
#include <tesseract_motion_planners/descartes/descartes_collision.h>
#include <tesseract_motion_planners/descartes/descartes_robot_positioner_sampler.h>
#include <tesseract_motion_planners/descartes/utils.h>
#include <tesseract_motion_planners/ompl/config/ompl_planner_config.h>
#include <tesseract/tesseract.h>

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <opw_kinematics/opw_parameters.h>
#include <Eigen/Geometry>
#include <descartes_samplers/evaluators/euclidean_distance_edge_evaluator.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/StateValidityChecker.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace twc_motion_planning
{
/**
 * @brief The Base ProcessPlannerConfig struct contains the information
 * necessary to construct a new planner instance
 */
struct ProcessPlannerConfig
{
  enum class ConfigType
  {
    NONE,
    ROBOT_ONLY,
    ROBOT_WITH_EXTERNAL_POSITIONER,
    ROBOT_ON_EXTERNAL_POSITIONER
  };

  /** @brief The configuration type */
  ConfigType type{ ConfigType::NONE };

  /** @brief The start position */
  tesseract_motion_planners::JointWaypoint::Ptr start;

  /** @brief The end position */
  tesseract_motion_planners::JointWaypoint::Ptr end;

  /** @brief The tesseract */
  tesseract::Tesseract::ConstPtr tesseract;

  /** @brief Manipulator name
   * It represents ROBOT_ONLY, ROBOT_WITH_EXTERNAL_POSITIONER, or ROBOT_ON_EXTERNAL_POSITIONER
   */
  std::string manipulator;

  /** @brief Link name that the TCP is associated with. Either on the robot or external positioner*/
  std::string link;

  /** @brief Robot only manipulator name. Does not include positioner */
  std::string robot;

  /** @brief The IK solver name for the robot only */
  std::string ik_solver;

  /** @brief Positioner manipulator name */
  std::string positioner;

  Eigen::Isometry3d tcp = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d local_offset = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d world_offset = Eigen::Isometry3d::Identity();
};

enum class RobotConfig
{
  NUT = 0,
  FUT = 1,
  NDT = 2,
  FDT = 3,
  NDB = 4,
  FDB = 5,
  NUB = 6,
  FUB = 7
};

/**
 * @brief Get the configuration of a six axis industrial robot
 * @param joint_values The joint values of the robot.
 * @param robot_kin The kinematics object of the robot.
 * @param sign_correction Correct the sign for Joint 3 and Joint 5 based on the robot manufacturer.
 * @return
 */
template <typename FloatType>
static inline RobotConfig
getRobotConfig(const Eigen::Ref<const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>>& joint_values,
               const tesseract_kinematics::ForwardKinematics::ConstPtr& robot_kin,
               const Eigen::Ref<const Eigen::Vector2i>& sign_correction = Eigen::Vector2i::Ones())
{
  // Get the pose at tool0
  Eigen::Isometry3d pose;
  robot_kin->calcFwdKin(pose, joint_values);

  // Get the base rotated by joint 1
  Eigen::Isometry3d prime_pose(Eigen::AngleAxisd(static_cast<double>(joint_values(0)), Eigen::Vector3d::UnitZ()));

  // Transform tool0 pose into new frame
  Eigen::Isometry3d pose_prime = prime_pose.inverse() * pose;

  // If pose_prime.x is greater than and equal to zero then it is in the forward configuration, otherwise
  // in the backward configuration.

  if ((sign_correction[1] * joint_values(4)) >= 0 && pose_prime.translation().x() >= 0 &&
      (sign_correction[0] * joint_values(2)) < M_PI / 2)
  {
    return RobotConfig::FUT;
  }
  else if ((sign_correction[1] * joint_values(4)) < 0 && pose_prime.translation().x() >= 0 &&
           (sign_correction[0] * joint_values(2)) < M_PI / 2)
  {
    return RobotConfig::NUT;
  }
  else if ((sign_correction[1] * joint_values(4)) >= 0 && pose_prime.translation().x() >= 0 &&
           (sign_correction[0] * joint_values(2)) >= M_PI / 2)
  {
    return RobotConfig::FDT;
  }
  else if ((sign_correction[1] * joint_values(4)) < 0 && pose_prime.translation().x() >= 0 &&
           (sign_correction[0] * joint_values(2)) >= M_PI / 2)
  {
    return RobotConfig::NDT;
  }
  else if ((sign_correction[1] * joint_values(4)) >= 0 && pose_prime.translation().x() < 0 &&
           (sign_correction[0] * joint_values(2)) < M_PI / 2)
  {
    return RobotConfig::FUB;
  }
  else if ((sign_correction[1] * joint_values(4)) < 0 && pose_prime.translation().x() < 0 &&
           (sign_correction[0] * joint_values(2)) < M_PI / 2)
  {
    return RobotConfig::NUB;
  }
  else if ((sign_correction[1] * joint_values(4)) >= 0 && pose_prime.translation().x() < 0 &&
           (sign_correction[0] * joint_values(2)) >= M_PI / 2)
  {
    return RobotConfig::FDB;
  }
  else  // if ((sign_correction[1] * joint_values(4)) < 0 && pose_prime.translation().x() < 0 && (sign_correction[0] *
        // joint_values(2)) >= M_PI / 2)
  {
    return RobotConfig::NDB;
  }
}

template <typename FloatType>
bool isWithinJointLimits(const Eigen::Ref<const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>>& joint_values,
                         const Eigen::Matrix<FloatType, Eigen::Dynamic, 2>& limits)
{
  for (int i = 0; i < limits.rows(); ++i)
    if ((joint_values(i) < limits(i, 0)) || (joint_values(i) > limits(i, 1)))
      return false;
  if (((joint_values(joint_values.size() - 3) + joint_values(joint_values.size() - 1)) < -300 * M_PI / 180.) ||
      ((joint_values(joint_values.size() - 3) + joint_values(joint_values.size() - 1)) > 300 * M_PI / 180.))
    return false;

  return true;
}

template <typename FloatType>
bool isValidState(const Eigen::Ref<const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>>& joint_values,
                  const Eigen::Matrix<FloatType, Eigen::Dynamic, 2>& limits,
                  const tesseract_kinematics::ForwardKinematics::ConstPtr& robot_kin)
{
  if (!isWithinJointLimits(joint_values, limits))
    return false;

  RobotConfig robot_config = getRobotConfig<FloatType>(joint_values.tail(robot_kin->numJoints()), robot_kin);
  if (robot_config != RobotConfig::FUT && robot_config != RobotConfig::NUT)
    return false;

  return true;
}

class TWCStateValidityChecker : public ompl::base::StateValidityChecker
{
public:
  TWCStateValidityChecker(const ompl::base::SpaceInformationPtr& si,
                          const tesseract_motion_planners::OMPLPlannerConfig& config,
                          tesseract_kinematics::ForwardKinematics::ConstPtr full_kin,
                          tesseract_kinematics::ForwardKinematics::ConstPtr robot_kin)
    : StateValidityChecker(si), full_kin_(std::move(full_kin)), robot_kin_(std::move(robot_kin))
  {
  }

  ~TWCStateValidityChecker() override = default;

  virtual bool isValid(const ompl::base::State* state) const
  {
    const ompl::base::RealVectorStateSpace::StateType* vector_state =
        state->as<ompl::base::RealVectorStateSpace::StateType>();

    auto jv = Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, 1>>(vector_state->values, full_kin_->numJoints());

    return (isValidState<double>(jv, full_kin_->getLimits(), robot_kin_));
  }

private:
  tesseract_kinematics::ForwardKinematics::ConstPtr full_kin_;
  tesseract_kinematics::ForwardKinematics::ConstPtr robot_kin_;
};

template <typename FloatType>
opw_kinematics::Parameters<FloatType> makeABBirb4600_60_205()
{
  opw_kinematics::Parameters<FloatType> p;
  p.a1 = FloatType(0.175);
  p.a2 = FloatType(-0.175);
  p.b = FloatType(0.000);
  p.c1 = FloatType(0.495);
  p.c2 = FloatType(0.9);
  p.c3 = FloatType(0.960);
  p.c4 = FloatType(0.135);

  p.offsets[2] = -M_PI / 2.0;

  return p;
}

template <typename FloatType>
std::vector<descartes_core::TimingConstraint<FloatType>>
makeTiming(const std::vector<tesseract_motion_planners::Waypoint::ConstPtr>& path, FloatType dt = 0.5)
{
  // TODO(jmeyer): Compute the real time
  // In Descartes land, the timing constraint represents how long the dt is between the previous point and the point
  // associated with this particular constraint. In a trajectory with only one pass the first point is meaningless (?).
  // Here I want to append many passes together so setting the DT to 0.0 is sort of saying: "Hey, take as long
  // as you need to get to here from the last point".
  std::vector<descartes_core::TimingConstraint<FloatType>> timing(path.size(), dt);

  return timing;
}

template <typename FloatType>
tesseract_motion_planners::DescartesMotionPlannerConfig<FloatType>
createDescartesPlannerConfig(const tesseract::Tesseract::ConstPtr& tesseract_ptr,
                             const Eigen::Ref<const Eigen::VectorXd>& positioner_sample_resolution,
                             const tesseract_kinematics::ForwardKinematics::ConstPtr& positioner_kin,
                             const tesseract_kinematics::ForwardKinematics::ConstPtr& full_kin,
                             const tesseract_kinematics::InverseKinematics::ConstPtr& robot_inv_kin,
                             const Eigen::Isometry3d& robot_tcp,
                             const double robot_reach,
                             const tesseract_environment::EnvState::ConstPtr& current_state,
                             const std::vector<tesseract_motion_planners::Waypoint::ConstPtr>& waypoints)
{
  ROS_INFO_STREAM("Configuring Descartes with " << waypoints.size() << " waypoints");
  auto check_kin = tesseract_ptr->getFwdKinematicsManagerConst()->getFwdKinematicSolver("robot_only");
  if (!tesseract_kinematics::checkKinematics(check_kin, robot_inv_kin))
    ROS_ERROR("Check Kinematics failed. This means that Inverse Kinematics does not agree with KDL (TrajOpt). "
              "Did you change the URDF recently?");

  std::vector<std::string> joint_names;
  const std::vector<std::string> positioner_joint_names = positioner_kin->getJointNames();
  const std::vector<std::string> robot_joint_names = robot_inv_kin->getJointNames();
  joint_names.insert(joint_names.end(), positioner_joint_names.begin(), positioner_joint_names.end());
  joint_names.insert(joint_names.end(), robot_joint_names.begin(), robot_joint_names.end());

  std::vector<std::string> active_link_names;
  const std::vector<std::string> positioner_active_link_names = positioner_kin->getActiveLinkNames();
  const std::vector<std::string> robot_active_link_names = robot_inv_kin->getActiveLinkNames();
  active_link_names.insert(
      active_link_names.end(), positioner_active_link_names.begin(), positioner_active_link_names.end());
  active_link_names.insert(active_link_names.end(), robot_active_link_names.begin(), robot_active_link_names.end());

  tesseract_environment::AdjacencyMap::Ptr adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(
      tesseract_ptr->getEnvironmentConst()->getSceneGraph(), active_link_names, current_state->transforms);

  // Get Collision Interface
  typename descartes_light::CollisionInterface<FloatType>::Ptr coll_interface =
      std::make_shared<tesseract_motion_planners::DescartesCollision<FloatType>>(
          tesseract_ptr->getEnvironmentConst(), adjacency_map->getActiveLinkNames(), joint_names);

  // Create Timing Constraint
  std::vector<descartes_core::TimingConstraint<FloatType>> timing =
      makeTiming(waypoints, std::numeric_limits<FloatType>::max());

  // Create Edge Evaluator
  typename descartes_light::EdgeEvaluator<FloatType>::Ptr edge_computer =
      std::make_shared<descartes_light::EuclideanDistanceEdgeEvaluator<FloatType>>(positioner_kin->numJoints() +
                                                                                   robot_inv_kin->numJoints());

  // Create isValid function pointer
  auto is_valid = std::bind(&twc_motion_planning::isValidState<FloatType>,
                            std::placeholders::_1,
                            full_kin->getLimits().cast<FloatType>(),
                            check_kin);

  // Create Position Samplers
  std::vector<typename descartes_light::PositionSampler<FloatType>::Ptr> position_samplers =
      tesseract_motion_planners::makeRobotPositionerSamplers<FloatType>(waypoints,
                                                                        positioner_kin,
                                                                        robot_inv_kin,
                                                                        coll_interface,
                                                                        current_state,
                                                                        positioner_sample_resolution,
                                                                        robot_tcp,
                                                                        robot_reach,
                                                                        true,
                                                                        is_valid);

  return tesseract_motion_planners::DescartesMotionPlannerConfig<FloatType>(tesseract_ptr,
                                                                            adjacency_map->getActiveLinkNames(),
                                                                            joint_names,
                                                                            edge_computer,
                                                                            timing,
                                                                            position_samplers,
                                                                            waypoints);
}

template <typename FloatType>
tesseract_motion_planners::DescartesMotionPlannerConfig<FloatType>
createDescartesPlannerConfig(const tesseract::Tesseract::ConstPtr& tesseract_ptr,
                             const tesseract_kinematics::ForwardKinematics::ConstPtr& full_kin,
                             const tesseract_kinematics::InverseKinematics::ConstPtr& robot_inv_kin,
                             const Eigen::Isometry3d& robot_tcp,
                             const double robot_reach,
                             const tesseract_environment::EnvState::ConstPtr& current_state,
                             const std::vector<tesseract_motion_planners::Waypoint::ConstPtr>& waypoints,
                             double contact_distance)
{
  ROS_INFO_STREAM("Configuring Descartes with " << waypoints.size() << " waypoints");
  auto check_kin = tesseract_ptr->getFwdKinematicsManagerConst()->getFwdKinematicSolver("robot_only");
  if (!tesseract_kinematics::checkKinematics(check_kin, robot_inv_kin))
    ROS_ERROR("Check Kinematics failed. This means that Inverse Kinematics does not agree with KDL (TrajOpt). "
              "Did you change the URDF recently?");

  std::vector<std::string> joint_names = robot_inv_kin->getJointNames();
  std::vector<std::string> active_link_names = robot_inv_kin->getActiveLinkNames();

  tesseract_environment::AdjacencyMap::Ptr adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(
      tesseract_ptr->getEnvironmentConst()->getSceneGraph(), active_link_names, current_state->transforms);

  // Get Collision Interface
  typename descartes_light::CollisionInterface<FloatType>::Ptr coll_interface =
      std::make_shared<tesseract_motion_planners::DescartesCollision<FloatType>>(
          tesseract_ptr->getEnvironmentConst(), adjacency_map->getActiveLinkNames(), joint_names, contact_distance);

  // Create Timing Constraint
  std::vector<descartes_core::TimingConstraint<FloatType>> timing =
      makeTiming(waypoints, std::numeric_limits<FloatType>::max());

  // Create Edge Evaluator
  typename descartes_light::EdgeEvaluator<FloatType>::Ptr edge_computer =
      std::make_shared<descartes_light::EuclideanDistanceEdgeEvaluator<FloatType>>(robot_inv_kin->numJoints());

  // Create isValid function pointer
  auto is_valid = std::bind(&twc_motion_planning::isValidState<FloatType>,
                            std::placeholders::_1,
                            full_kin->getLimits().cast<FloatType>(),
                            check_kin);

  // Create Position Samplers
  std::vector<typename descartes_light::PositionSampler<FloatType>::Ptr> position_samplers =
      tesseract_motion_planners::makeRobotSamplers<FloatType>(waypoints,
                                                              robot_inv_kin,
                                                              coll_interface,
                                                              current_state,
                                                              robot_tcp,
                                                              robot_reach,
                                                              true,
                                                              is_valid,
                                                              0.001,
                                                              0.001,
                                                              0.001,
                                                              10 * M_PI / 180,
                                                              10 * M_PI / 180,
                                                              10 * M_PI / 180);

  return tesseract_motion_planners::DescartesMotionPlannerConfig<FloatType>(tesseract_ptr,
                                                                            adjacency_map->getActiveLinkNames(),
                                                                            joint_names,
                                                                            edge_computer,
                                                                            timing,
                                                                            position_samplers,
                                                                            waypoints);
}

}  // namespace twc_motion_planning
#endif  // TWC_MOTION_PLANNING_UTILS_H
