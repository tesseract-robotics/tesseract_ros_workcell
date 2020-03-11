#include <tesseract_process_planners/generators/axial_approach_generator.h>
#include <tesseract_process_planners/generators/axial_departure_generator.h>
#include <tesseract_process_planners/generators/passthrough_process_generator.h>
#include <tesseract_process_planners/generators/linear_transition_generator.h>
#include <tesseract_process_planners/generators/passthrough_transition_generator.h>
#include <tesseract_motion_planners/trajopt/config/trajopt_planner_default_config.h>
#include <tesseract_motion_planners/trajopt/config/trajopt_planner_freespace_config.h>
#include <tesseract_motion_planners/descartes/descartes_motion_planner.h>
#include <tesseract_environment/core/environment.h>
#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_motion_planners/core/waypoint.h>
#include <tesseract/tesseract.h>

#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <tesseract_motion_planners/ompl/continuous_motion_validator.h>
#include <tesseract_motion_planners/ompl/config/ompl_planner_freespace_config.h>
#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>
#include <tesseract_motion_planners/ompl/ompl_planner_configurator.h>

#include <twc_motion_planning/sanding_definition_generator.h>

#include <ros/console.h>

#include <twc_motion_planning/sanding_process_planner.h>

static const double LONGEST_VALID_SEGMENT_FRACTION = 0.01;
static const double LONGEST_VALID_SEGMENT_LENGTH = 0.08;
static const bool PLANNING_VERBOSE = true;
static const bool USE_FULL_SEED = true;
static const double PROCESS_CONTACT_DISTANCE = 0.005;
static const double FREESPACE_CONTACT_DISTANCE = 0.01;

using DescartesFloatType = double;

namespace twc_motion_planning
{
SandingProcessPlanner::SandingProcessPlanner(ProcessPlannerConfig config) : config_(config) {}

const tesseract_process_planners::ProcessDefinition& SandingProcessPlanner::getProcessDefinition() const
{
  return process_definition_;
}

void SandingProcessPlanner::setToolPath(
    const std::vector<std::vector<tesseract_motion_planners::Waypoint::Ptr>>& tool_path)
{
  tesseract_process_planners::SandingDefinitionGenerator sanding_definition_generator;
  sanding_definition_generator.setLocalOffset(config_.local_offset);
  sanding_definition_generator.setWorldOffset(config_.world_offset);
  sanding_definition_generator.setStartWaypoint(config_.start);
  sanding_definition_generator.setEndWaypoint(config_.start);

  process_definition_ = sanding_definition_generator.generate(tool_path);
}

tesseract_process_planners::ProcessPlan SandingProcessPlanner::plan()
{
  Eigen::VectorXd seed_state;
  seed_state.resize(
      config_.tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver(config_.manipulator)->numJoints());
  seed_state.setZero();

  return plan(seed_state);
}

tesseract_process_planners::ProcessPlan SandingProcessPlanner::plan(const Eigen::VectorXd& seed)
{
  process_plan_ = tesseract_process_planners::ProcessPlan();
  process_plan_.segments.reserve(process_definition_.segments.size());
  process_plan_.transition_from_start.resize(process_definition_.segments.size() - 1);
  process_plan_.transition_from_end.resize(process_definition_.segments.size() - 1);
  Eigen::VectorXd seed_state = seed;

  tesseract_kinematics::ForwardKinematics::ConstPtr full_kin =
      config_.tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver(config_.manipulator);
  tesseract_kinematics::ForwardKinematics::ConstPtr robot_kin =
      config_.tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver(config_.robot);

  if (USE_FULL_SEED)
  {
    seed_plan_ = generateDenseSeedTrajectoryRobotOnly();
    // Check status
    if (!seed_plan_.valid)
    {
      ROS_ERROR("Failed to find global seed trajectory!");
      process_plan_.valid = false;
      return process_plan_;
    }
  }

  // Generate all process segment trajectories
  for (std::size_t idx = 0; idx < process_definition_.segments.size(); ++idx)
  {
    tesseract_process_planners::ProcessSegmentPlan segment_plan;
    tesseract_motion_planners::PlannerResponse segment_plan_response;

    // Create motion plan for process
    auto segment_process = generateSegmentProcessTrajectoryRobotOnly(idx);

    segment_plan = segment_process.first;
    segment_plan_response = segment_process.second;

    // Check status
    if (!segment_plan_response.status)
    {
      ROS_ERROR("Failed to find motion plan for a segment %d process: %s",
                idx,
                segment_plan_response.status.message().c_str());
      segment_plan.valid = false;
      process_plan_.valid = false;
      return process_plan_;
    }

    // Create motion plan for approach
    if (!process_definition_.segments[idx].approach.empty())
    {
      generateSegmentApproachTrajectoryRobotOnly(segment_plan, idx);

      if (!segment_plan.valid)
      {
        process_plan_.valid = false;
        return process_plan_;
      }
    }

    // Create motion plan for departure
    if (!process_definition_.segments[idx].departure.empty())
    {
      generateSegmentDepartureTrajectoryRobotOnly(segment_plan, idx);

      if (!segment_plan.valid)
      {
        process_plan_.valid = false;
        return process_plan_;
      }
    }

    process_plan_.segments.push_back(segment_plan);
  }

  if (process_plan_.segments.size() != process_definition_.segments.size())
  {
    ROS_ERROR("Failed to find motion plans for all segments!");
    process_plan_.valid = false;
    return process_plan_;
  }

  // Generate transition from home to first segment
  if (!process_plan_.segments.empty() && process_plan_.segments.front().valid)
  {
    tesseract_motion_planners::PlannerResponse from_home_planner_response;
    tesseract_motion_planners::JointWaypoint::Ptr end_waypoint;
    if (process_plan_.segments.front().approach.trajectory.size() != 0)
      end_waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>(
          process_plan_.segments.front().approach.trajectory.topRows(1).transpose(), full_kin->getJointNames());
    else
      end_waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>(
          process_plan_.segments.front().process.trajectory.topRows(1).transpose(), full_kin->getJointNames());
    computeFreeSpaceMotion(from_home_planner_response, config_.end, end_waypoint);
    // Check status
    if (!from_home_planner_response.status)
    {
      process_plan_.valid = false;
      return process_plan_;
    }
    process_plan_.from_start = from_home_planner_response.joint_trajectory;
  }
  else
  {
    ROS_ERROR("Unable to plan for from home transition because first segment faild to find a valid motion plan!");
  }

  // Generate transition from last segment to home
  if (process_plan_.segments.back().valid)
  {
    tesseract_motion_planners::PlannerResponse to_home_planner_response;
    tesseract_motion_planners::JointWaypoint::Ptr end_waypoint;
    if (process_plan_.segments.back().departure.trajectory.size() != 0)
      end_waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>(
          process_plan_.segments.back().departure.trajectory.bottomRows(1).transpose(), full_kin->getJointNames());
    else
      end_waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>(
          process_plan_.segments.back().process.trajectory.bottomRows(1).transpose(), full_kin->getJointNames());
    computeFreeSpaceMotion(to_home_planner_response, end_waypoint, config_.end);
    // Check status
    if (!to_home_planner_response.status)
    {
      process_plan_.valid = false;
      return process_plan_;
    }
    process_plan_.to_end = to_home_planner_response.joint_trajectory;
  }
  else
  {
    ROS_ERROR("Unable to plan for to home transition because last segment faild to find a valid motion plan!");
  }

  // Generate segment transitions
  for (size_t i = 0; i < (process_definition_.segments.size() - 1); ++i)
  {
    generateSegmentTransitionTrajectoryRobotOnly(i);
    if (!process_plan_.valid)
      return process_plan_;
  }

  return process_plan_;
}

std::pair<tesseract_process_planners::ProcessSegmentPlan, tesseract_motion_planners::PlannerResponse>
SandingProcessPlanner::generateSegmentSeedTrajectory(std::size_t segment_idx)
{
  // Get world to link transforms
  std::vector<double> joint_values(config_.tesseract->getEnvironmentConst()->getActiveJointNames().size(), 0);
  tesseract_environment::EnvState::Ptr current_state = config_.tesseract->getEnvironmentConst()->getState(
      config_.tesseract->getEnvironmentConst()->getActiveJointNames(), joint_values);
  Eigen::Isometry3d change_base = current_state->transforms[config_.tesseract->getFwdKinematicsManagerConst()
                                                                ->getFwdKinematicSolver(config_.manipulator)
                                                                ->getBaseLinkName()];

  // Flatten process segment
  std::vector<tesseract_motion_planners::Waypoint::ConstPtr> flattened_process_definition;
  for (const auto& wp : process_definition_.segments[segment_idx])
    flattened_process_definition.push_back(wp);

  // Setup Descartes planner
  tesseract_process_planners::ProcessSegmentPlan seed_process_segment_plan;
  tesseract_motion_planners::PlannerResponse descartes_planner_response;
  tesseract_motion_planners::DescartesMotionPlanner<double> cartesian_descartes_planner;

  // Setup Descartes planner
  Eigen::Matrix<DescartesFloatType, 1, 1> rail_sample_resolution;
  rail_sample_resolution << 0.05;

  tesseract_kinematics::ForwardKinematics::ConstPtr positioner_kin =
      config_.tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver("positioner");
  tesseract_kinematics::ForwardKinematics::ConstPtr full_kin =
      config_.tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver("manipulator");
  tesseract_kinematics::InverseKinematics::ConstPtr robot_inv_kin =
      config_.tesseract->getInvKinematicsManagerConst()->getInvKinematicSolver("robot_only", "OPWInvKin");

  tesseract_motion_planners::DescartesMotionPlannerConfig<DescartesFloatType> descartes_config =
      createDescartesPlannerConfig<DescartesFloatType>(config_.tesseract,
                                                       rail_sample_resolution,
                                                       positioner_kin,
                                                       full_kin,
                                                       robot_inv_kin,
                                                       config_.tcp,
                                                       2.5,
                                                       current_state,
                                                       flattened_process_definition);
  cartesian_descartes_planner.setConfiguration(descartes_config);
  cartesian_descartes_planner.solve(descartes_planner_response);

  // If it succeeded ignoring if it is in collision
  if (!descartes_planner_response.status &&
      descartes_planner_response.status.value() !=
          tesseract_motion_planners::DescartesMotionPlannerStatusCategory::ErrorFoundValidSolutionInCollision)
  {
    ROS_ERROR("Failed to find seed motion plan for process, found %lu failed vertices: %s",
              descartes_planner_response.failed_waypoints.size(),
              descartes_planner_response.status.message().c_str());

    seed_process_segment_plan.valid = false;
    return std::make_pair(seed_process_segment_plan, descartes_planner_response);
  }

  long num_rows = 0;
  long idx = 0;

  seed_process_segment_plan.valid = true;

  num_rows = static_cast<long>(process_definition_.segments[segment_idx].approach.size());
  seed_process_segment_plan.approach.joint_names = descartes_planner_response.joint_trajectory.joint_names;
  seed_process_segment_plan.approach.trajectory =
      descartes_planner_response.joint_trajectory.trajectory.topRows(idx + num_rows).bottomRows(num_rows);
  idx += num_rows;

  num_rows = static_cast<long>(process_definition_.segments[segment_idx].process.size());
  seed_process_segment_plan.process.joint_names = descartes_planner_response.joint_trajectory.joint_names;
  seed_process_segment_plan.process.trajectory =
      descartes_planner_response.joint_trajectory.trajectory.topRows(idx + num_rows).bottomRows(num_rows);
  idx += num_rows;

  num_rows = static_cast<long>(process_definition_.segments[segment_idx].departure.size());
  seed_process_segment_plan.departure.joint_names = descartes_planner_response.joint_trajectory.joint_names;
  seed_process_segment_plan.departure.trajectory =
      descartes_planner_response.joint_trajectory.trajectory.topRows(idx + num_rows).bottomRows(num_rows);
  idx += num_rows;

  return std::make_pair(seed_process_segment_plan, descartes_planner_response);
}

std::pair<tesseract_process_planners::ProcessSegmentPlan, tesseract_motion_planners::PlannerResponse>
SandingProcessPlanner::generateSegmentProcessTrajectory(std::size_t segment_idx)
{
  tesseract_kinematics::ForwardKinematics::ConstPtr full_kin =
      config_.tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver(config_.manipulator);

  auto config = std::make_shared<tesseract_motion_planners::TrajOptPlannerDefaultConfig>(
      config_.tesseract, config_.manipulator, config_.link, config_.tcp);
  config->init_type = trajopt::InitInfo::GIVEN_TRAJ;
  config->longest_valid_segment_fraction = LONGEST_VALID_SEGMENT_FRACTION;
  config->params.max_iter = 50;
  config->optimizer = sco::ModelType::OSQP;
  config->velocity_coeff = Eigen::VectorXd::Ones(full_kin->numJoints());
  config->acceleration_coeff.Ones(full_kin->numJoints());
  config->jerk_coeff.Ones(full_kin->numJoints());
  config->collision_cost_config.enabled = false;
  config->collision_constraint_config.enabled = true;
  config->collision_constraint_config.type = trajopt::CollisionEvaluatorType::CAST_CONTINUOUS;

  tesseract_process_planners::ProcessSegmentPlan segment_plan;
  tesseract_motion_planners::PlannerResponse seed_segment_process_plan_response;

  ROS_INFO("Starting segment seed trajectory generation.");
  auto seed = generateSegmentSeedTrajectory(segment_idx);
  ROS_INFO("Finished segment seed trajectory generation.");

  segment_plan = seed.first;
  seed_segment_process_plan_response = seed.second;

  if (!seed_segment_process_plan_response.status &&
      seed_segment_process_plan_response.status.value() !=
          tesseract_motion_planners::DescartesMotionPlannerStatusCategory::ErrorFoundValidSolutionInCollision)
  {
    ROS_INFO("Failed to find valid seed trajectory using descartes.");
    return std::make_pair(segment_plan, seed_segment_process_plan_response);
  }

  // Update the config target waypoints and seed trajectory
  config->target_waypoints = process_definition_.segments[segment_idx].process;
  config->seed_trajectory = segment_plan.process.trajectory;

  // Create motion plan for process
  tesseract_motion_planners::PlannerResponse process_planner_response;
  tesseract_motion_planners::TrajOptMotionPlanner trajopt_planner;

  ROS_INFO("Starting trajopt trajectory generation.");
  trajopt_planner.setConfiguration(config);
  trajopt_planner.solve(process_planner_response, PLANNING_VERBOSE);
  ROS_INFO("Finished trajopt trajectory generation.");

  segment_plan.process = process_planner_response.joint_trajectory;

  return std::make_pair(segment_plan, process_planner_response);
}

std::pair<tesseract_process_planners::ProcessSegmentPlan, tesseract_motion_planners::PlannerResponse>
SandingProcessPlanner::generateSegmentSeedTrajectoryRobotOnly(std::size_t segment_idx)
{
  // Get world to link transforms
  std::vector<double> joint_values(config_.tesseract->getEnvironmentConst()->getActiveJointNames().size(), 0);
  tesseract_environment::EnvState::Ptr current_state = config_.tesseract->getEnvironmentConst()->getState(
      config_.tesseract->getEnvironmentConst()->getActiveJointNames(), joint_values);
  Eigen::Isometry3d change_base = current_state->transforms[config_.tesseract->getFwdKinematicsManagerConst()
                                                                ->getFwdKinematicSolver(config_.manipulator)
                                                                ->getBaseLinkName()];

  // Flatten process segment
  std::vector<tesseract_motion_planners::Waypoint::ConstPtr> flattened_process_definition;
  for (const auto& wp : process_definition_.segments[segment_idx])
    flattened_process_definition.push_back(wp);

  // Setup Descartes planner
  tesseract_process_planners::ProcessSegmentPlan seed_process_segment_plan;
  tesseract_motion_planners::PlannerResponse descartes_planner_response;
  tesseract_motion_planners::DescartesMotionPlanner<DescartesFloatType> cartesian_descartes_planner;

  // Setup Descartes planner
  tesseract_kinematics::ForwardKinematics::ConstPtr full_kin =
      config_.tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver(config_.manipulator);
  tesseract_kinematics::InverseKinematics::ConstPtr robot_inv_kin =
      config_.tesseract->getInvKinematicsManagerConst()->getInvKinematicSolver(config_.robot,
                                                                               config_.ik_solver);  // "OPWInvKin"

  tesseract_motion_planners::DescartesMotionPlannerConfig<DescartesFloatType> descartes_config =
      createDescartesPlannerConfig<DescartesFloatType>(config_.tesseract,
                                                       full_kin,
                                                       robot_inv_kin,
                                                       config_.tcp,
                                                       2.5,
                                                       current_state,
                                                       flattened_process_definition,
                                                       PROCESS_CONTACT_DISTANCE);
  cartesian_descartes_planner.setConfiguration(descartes_config);
  cartesian_descartes_planner.solve(descartes_planner_response);

  // If it succeeded ignoring if it is in collision
  if (!descartes_planner_response.status &&
      descartes_planner_response.status.value() !=
          tesseract_motion_planners::DescartesMotionPlannerStatusCategory::ErrorFoundValidSolutionInCollision)
  {
    ROS_ERROR("Failed to find seed motion plan for process, found %lu failed vertices: %s",
              descartes_planner_response.failed_waypoints.size(),
              descartes_planner_response.status.message().c_str());

    seed_process_segment_plan.valid = false;
    return std::make_pair(seed_process_segment_plan, descartes_planner_response);
  }

  long num_rows = 0;
  long idx = 0;

  seed_process_segment_plan.valid = true;

  num_rows = static_cast<long>(process_definition_.segments[segment_idx].approach.size());
  seed_process_segment_plan.approach.joint_names = descartes_planner_response.joint_trajectory.joint_names;
  seed_process_segment_plan.approach.trajectory =
      descartes_planner_response.joint_trajectory.trajectory.topRows(idx + num_rows).bottomRows(num_rows);
  idx += num_rows;

  num_rows = static_cast<long>(process_definition_.segments[segment_idx].process.size());
  seed_process_segment_plan.process.joint_names = descartes_planner_response.joint_trajectory.joint_names;
  seed_process_segment_plan.process.trajectory =
      descartes_planner_response.joint_trajectory.trajectory.topRows(idx + num_rows).bottomRows(num_rows);
  idx += num_rows;

  num_rows = static_cast<long>(process_definition_.segments[segment_idx].departure.size());
  seed_process_segment_plan.departure.joint_names = descartes_planner_response.joint_trajectory.joint_names;
  seed_process_segment_plan.departure.trajectory =
      descartes_planner_response.joint_trajectory.trajectory.topRows(idx + num_rows).bottomRows(num_rows);
  idx += num_rows;

  return std::make_pair(seed_process_segment_plan, descartes_planner_response);
}

std::pair<tesseract_process_planners::ProcessSegmentPlan, tesseract_motion_planners::PlannerResponse>
SandingProcessPlanner::generateSegmentProcessTrajectoryRobotOnly(std::size_t segment_idx)
{
  tesseract_kinematics::ForwardKinematics::ConstPtr full_kin =
      config_.tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver(config_.manipulator);

  auto config = std::make_shared<tesseract_motion_planners::TrajOptPlannerDefaultConfig>(
      config_.tesseract, config_.manipulator, config_.link, config_.tcp);
  config->init_type = trajopt::InitInfo::GIVEN_TRAJ;
  config->longest_valid_segment_fraction = LONGEST_VALID_SEGMENT_FRACTION;
  config->params.max_iter = 50;
  config->optimizer = sco::ModelType::OSQP;
  config->velocity_coeff = Eigen::VectorXd::Ones(full_kin->numJoints());
  config->acceleration_coeff.Ones(full_kin->numJoints());
  config->jerk_coeff.Ones(full_kin->numJoints());
  config->collision_cost_config.enabled = false;
  config->collision_constraint_config.enabled = true;
  config->collision_constraint_config.type = trajopt::CollisionEvaluatorType::CAST_CONTINUOUS;

  tesseract_process_planners::ProcessSegmentPlan segment_plan;

  if (!USE_FULL_SEED)
  {
    tesseract_motion_planners::PlannerResponse seed_segment_process_plan_response;

    ROS_INFO("Starting segment seed trajectory generation.");
    auto seed = generateSegmentSeedTrajectoryRobotOnly(segment_idx);
    ROS_INFO("Finished segment seed trajectory generation.");

    segment_plan = seed.first;
    seed_segment_process_plan_response = seed.second;

    if (!seed_segment_process_plan_response.status &&
        seed_segment_process_plan_response.status.value() !=
            tesseract_motion_planners::DescartesMotionPlannerStatusCategory::ErrorFoundValidSolutionInCollision)
    {
      ROS_INFO("Failed to find valid seed trajectory using descartes.");
      return std::make_pair(segment_plan, seed_segment_process_plan_response);
    }
  }
  else
  {
    segment_plan = seed_plan_.segments[segment_idx];
  }

  // Update the config target waypoints and seed trajectory
  config->target_waypoints = process_definition_.segments[segment_idx].process;
  config->seed_trajectory = segment_plan.process.trajectory;

  // Create motion plan for process
  tesseract_motion_planners::PlannerResponse process_planner_response;
  tesseract_motion_planners::TrajOptMotionPlanner trajopt_planner;

  ROS_INFO("Starting trajopt trajectory generation.");
  trajopt_planner.setConfiguration(config);
  trajopt_planner.solve(process_planner_response, PLANNING_VERBOSE);
  ROS_INFO("Finished trajopt trajectory generation.");

  segment_plan.process = process_planner_response.joint_trajectory;

  return std::make_pair(segment_plan, process_planner_response);
}

void SandingProcessPlanner::generateSegmentApproachTrajectoryRobotOnly(
    tesseract_process_planners::ProcessSegmentPlan& segment_plan,
    std::size_t segment_idx)
{
  tesseract_kinematics::ForwardKinematics::ConstPtr full_kin =
      config_.tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver(config_.manipulator);

  auto config = std::make_shared<tesseract_motion_planners::TrajOptPlannerDefaultConfig>(
      config_.tesseract, config_.manipulator, config_.link, config_.tcp);
  config->init_type = trajopt::InitInfo::GIVEN_TRAJ;
  config->longest_valid_segment_fraction = LONGEST_VALID_SEGMENT_FRACTION;
  config->params.max_iter = 50;
  config->optimizer = sco::ModelType::OSQP;
  config->velocity_coeff = Eigen::VectorXd::Ones(full_kin->numJoints());
  config->acceleration_coeff.Ones(full_kin->numJoints());
  config->jerk_coeff.Ones(full_kin->numJoints());
  config->collision_cost_config.enabled = false;
  config->collision_constraint_config.enabled = true;
  config->collision_constraint_config.type = trajopt::CollisionEvaluatorType::CAST_CONTINUOUS;

  // Update Seed state
  auto seed_state = segment_plan.process.trajectory.topRows(1).transpose();
  tesseract_motion_planners::PlannerResponse approach_planner_response;
  tesseract_motion_planners::TrajOptMotionPlanner trajopt_planner;

  config->target_waypoints = process_definition_.segments[segment_idx].approach;
  config->seed_trajectory = segment_plan.approach.trajectory;
  config->seed_trajectory.bottomRows(1) = segment_plan.process.trajectory.topRows(1);
  // Need to force the end waypoint to be a joint waypoint based on the process motion plan found.
  tesseract_motion_planners::JointWaypoint::Ptr end_waypoint =
      std::make_shared<tesseract_motion_planners::JointWaypoint>(seed_state, full_kin->getJointNames());
  config->target_waypoints.back() = end_waypoint;

  trajopt_planner.setConfiguration(config);
  trajopt_planner.solve(approach_planner_response, PLANNING_VERBOSE);

  // Check status
  if (!approach_planner_response.status)
  {
    ROS_ERROR("Failed to find motion plan for a segment %d approach: %s",
              segment_idx,
              approach_planner_response.status.message().c_str());
    segment_plan.valid = false;
    return;
  }
  segment_plan.approach = approach_planner_response.joint_trajectory;
}

void SandingProcessPlanner::generateSegmentDepartureTrajectoryRobotOnly(
    tesseract_process_planners::ProcessSegmentPlan& segment_plan,
    std::size_t segment_idx)
{
  tesseract_kinematics::ForwardKinematics::ConstPtr full_kin =
      config_.tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver(config_.manipulator);

  auto config = std::make_shared<tesseract_motion_planners::TrajOptPlannerDefaultConfig>(
      config_.tesseract, config_.manipulator, config_.link, config_.tcp);
  config->init_type = trajopt::InitInfo::GIVEN_TRAJ;
  config->longest_valid_segment_fraction = LONGEST_VALID_SEGMENT_FRACTION;
  config->params.max_iter = 50;
  config->optimizer = sco::ModelType::OSQP;
  config->velocity_coeff = Eigen::VectorXd::Ones(full_kin->numJoints());
  config->acceleration_coeff.Ones(full_kin->numJoints());
  config->jerk_coeff.Ones(full_kin->numJoints());
  config->collision_cost_config.enabled = false;
  config->collision_constraint_config.enabled = true;
  config->collision_constraint_config.type = trajopt::CollisionEvaluatorType::CAST_CONTINUOUS;

  // Update Seed state
  auto seed_state = segment_plan.process.trajectory.bottomRows(1).transpose();
  tesseract_motion_planners::PlannerResponse departure_planner_response;
  tesseract_motion_planners::TrajOptMotionPlanner trajopt_planner;

  config->target_waypoints = process_definition_.segments[segment_idx].departure;
  config->seed_trajectory = segment_plan.departure.trajectory;
  config->seed_trajectory.topRows(1) = segment_plan.process.trajectory.bottomRows(1);

  // Need to force the start waypoint to be a joint waypoint based on the process motion plan found.
  tesseract_motion_planners::JointWaypoint::Ptr start_waypoint =
      std::make_shared<tesseract_motion_planners::JointWaypoint>(seed_state, full_kin->getJointNames());
  config->target_waypoints.front() = start_waypoint;

  trajopt_planner.setConfiguration(config);
  trajopt_planner.solve(departure_planner_response, PLANNING_VERBOSE);

  // Check status
  if (!departure_planner_response.status)
  {
    ROS_ERROR("Failed to find motion plan for a segment %d departure: %s",
              segment_idx,
              departure_planner_response.status.message().c_str());
    segment_plan.valid = false;
    return;
  }
  segment_plan.departure = departure_planner_response.joint_trajectory;
}

void SandingProcessPlanner::generateSegmentTransitionTrajectoryRobotOnly(std::size_t segment_idx)
{
  tesseract_motion_planners::JointWaypoint::Ptr s1_start_waypoint, s1_end_waypoint, s2_start_waypoint, s2_end_waypoint;

  tesseract_kinematics::ForwardKinematics::ConstPtr full_kin =
      config_.tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver(config_.manipulator);

  // Set start and end waypoints depending on if the approach/depart exists
  if (process_plan_.segments[segment_idx].approach.trajectory.size() != 0)
    s1_start_waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>(
        process_plan_.segments[segment_idx].approach.trajectory.topRows(1).transpose(), full_kin->getJointNames());
  else
    s1_start_waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>(
        process_plan_.segments[segment_idx].process.trajectory.topRows(1).transpose(), full_kin->getJointNames());
  if (process_plan_.segments[segment_idx].departure.trajectory.size() != 0)
    s1_end_waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>(
        process_plan_.segments[segment_idx].departure.trajectory.bottomRows(1).transpose(), full_kin->getJointNames());
  else
    s1_end_waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>(
        process_plan_.segments[segment_idx].process.trajectory.bottomRows(1).transpose(), full_kin->getJointNames());
  if (process_plan_.segments[segment_idx + 1].approach.trajectory.size() != 0)
    s2_start_waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>(
        process_plan_.segments[segment_idx + 1].approach.trajectory.topRows(1).transpose(), full_kin->getJointNames());
  else
    s2_start_waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>(
        process_plan_.segments[segment_idx + 1].process.trajectory.topRows(1).transpose(), full_kin->getJointNames());
  if (process_plan_.segments[segment_idx + 1].departure.trajectory.size() != 0)
    s2_end_waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>(
        process_plan_.segments[segment_idx + 1].departure.trajectory.bottomRows(1).transpose(),
        full_kin->getJointNames());
  else
    s2_end_waypoint = std::make_shared<tesseract_motion_planners::JointWaypoint>(
        process_plan_.segments[segment_idx + 1].process.trajectory.bottomRows(1).transpose(),
        full_kin->getJointNames());

  // Create motion plan for transition from start
  tesseract_motion_planners::PlannerResponse transition_from_start_planner_response;
  computeFreeSpaceMotion(transition_from_start_planner_response, s1_start_waypoint, s2_end_waypoint);

  // Check status
  if (!transition_from_start_planner_response.status)
  {
    ROS_WARN("Failed to find transition from start of segment: %s",
             transition_from_start_planner_response.status.message().c_str());
  }
  process_plan_.transition_from_start[segment_idx] = transition_from_start_planner_response.joint_trajectory;

  // Create motion plan for transition from start
  tesseract_motion_planners::PlannerResponse transition_from_end_planner_response;
  computeFreeSpaceMotion(transition_from_end_planner_response, s1_end_waypoint, s2_start_waypoint);

  // Check status
  if (!transition_from_end_planner_response.status)
  {
    ROS_ERROR("Failed to find transition from end of segment: %s",
              transition_from_end_planner_response.status.message().c_str());
    process_plan_.valid = false;
    return;
  }
  process_plan_.transition_from_end[segment_idx] = transition_from_end_planner_response.joint_trajectory;
}

tesseract_process_planners::ProcessPlan SandingProcessPlanner::generateDenseSeedTrajectoryRobotOnly()
{
  // Get world to link transforms
  std::vector<double> joint_values(config_.tesseract->getEnvironmentConst()->getActiveJointNames().size(), 0);
  tesseract_environment::EnvState::Ptr current_state = config_.tesseract->getEnvironmentConst()->getState(
      config_.tesseract->getEnvironmentConst()->getActiveJointNames(), joint_values);
  Eigen::Isometry3d change_base = current_state->transforms[config_.tesseract->getFwdKinematicsManagerConst()
                                                                ->getFwdKinematicSolver(config_.manipulator)
                                                                ->getBaseLinkName()];

  // Generate all process segment trajectories using Descartes
  std::vector<tesseract_motion_planners::Waypoint::ConstPtr> flattened_process_definition;
  tesseract_motion_planners::JointWaypoint::ConstPtr home_pose =
      std::static_pointer_cast<const tesseract_motion_planners::JointWaypoint>(process_definition_.start);
  tesseract_motion_planners::CartesianWaypoint::ConstPtr first_process_waypoint;
  if (!process_definition_.segments.front().approach.empty() &&
      (process_definition_.segments.front().approach.front() != nullptr))
    first_process_waypoint = std::static_pointer_cast<const tesseract_motion_planners::CartesianWaypoint>(
        process_definition_.segments.front().approach.front());
  else
    first_process_waypoint = std::static_pointer_cast<const tesseract_motion_planners::CartesianWaypoint>(
        process_definition_.segments.front().process.front());
  tesseract_motion_planners::CartesianWaypoint::ConstPtr last_process_waypoint;
  if (!process_definition_.segments.back().departure.empty() &&
      (process_definition_.segments.back().departure.back() != nullptr))
    last_process_waypoint = std::static_pointer_cast<const tesseract_motion_planners::CartesianWaypoint>(
        process_definition_.segments.back().departure.back());
  else
    last_process_waypoint = std::static_pointer_cast<const tesseract_motion_planners::CartesianWaypoint>(
        process_definition_.segments.back().process.back());

  Eigen::Isometry3d home_transform;
  config_.tesseract->getFwdKinematicsManagerConst()
      ->getFwdKinematicSolver(config_.manipulator)
      ->calcFwdKin(home_transform, home_pose->getPositions());
  home_transform = change_base * home_transform;
  home_transform = home_transform * config_.tcp;

  auto home_cart_waypoint = std::make_shared<tesseract_motion_planners::CartesianWaypoint>(home_transform);
  home_cart_waypoint->setCoefficients(first_process_waypoint->getCoefficients());
  home_cart_waypoint->setIsCritical(first_process_waypoint->isCritical());

  int from_home_steps = std::max(
      static_cast<int>((home_cart_waypoint->getPosition() - first_process_waypoint->getPosition()).norm() / 0.2) + 1,
      5);
  int to_home_steps = std::max(
      static_cast<int>((home_cart_waypoint->getPosition() - last_process_waypoint->getPosition()).norm() / 0.2) + 1, 5);

  std::vector<tesseract_motion_planners::Waypoint::Ptr> from_home_waypoints =
      tesseract_motion_planners::interpolate(*home_cart_waypoint, *first_process_waypoint, from_home_steps);
  std::vector<tesseract_motion_planners::Waypoint::Ptr> to_home_waypoints =
      tesseract_motion_planners::interpolate(*last_process_waypoint, *home_cart_waypoint, to_home_steps);

  for (const auto& wp : from_home_waypoints)
    flattened_process_definition.push_back(wp);

  // Need to set start and last waypoint back to home (joint pose) so descartes does not sample alternative joint
  // position
  flattened_process_definition.front() = home_pose;

  for (size_t i = 0; i < process_definition_.segments.size(); ++i)
  {
    for (const auto& wp : process_definition_.segments[i].approach)
      flattened_process_definition.push_back(wp);

    for (const auto& wp : process_definition_.segments[i].process)
      flattened_process_definition.push_back(wp);

    for (const auto& wp : process_definition_.segments[i].departure)
      flattened_process_definition.push_back(wp);

    if (i < (process_definition_.segments.size() - 1))
      for (const auto& wp : process_definition_.transitions[i].transition_from_end)
        flattened_process_definition.push_back(wp);
  }

  for (const auto& wp : to_home_waypoints)
    flattened_process_definition.push_back(wp);

  // Need to set start and last waypoint back to home (joint pose) so descartes does not sample alternative joint
  // position
  flattened_process_definition.back() = home_pose;

  // Setup Descartes planner
  tesseract_kinematics::ForwardKinematics::ConstPtr full_kin =
      config_.tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver(config_.manipulator);
  tesseract_kinematics::InverseKinematics::ConstPtr robot_inv_kin =
      config_.tesseract->getInvKinematicsManagerConst()->getInvKinematicSolver(config_.robot,
                                                                               config_.ik_solver);  // "OPWInvKin"

  tesseract_motion_planners::DescartesMotionPlannerConfig<DescartesFloatType> descartes_config =
      createDescartesPlannerConfig<DescartesFloatType>(config_.tesseract,
                                                       full_kin,
                                                       robot_inv_kin,
                                                       config_.tcp,
                                                       2.5,
                                                       current_state,
                                                       flattened_process_definition,
                                                       PROCESS_CONTACT_DISTANCE);

  tesseract_process_planners::ProcessPlan seed_process_plan;
  tesseract_motion_planners::PlannerResponse descartes_planner_response;
  tesseract_motion_planners::DescartesMotionPlanner<DescartesFloatType> cartesian_descartes_planner;

  cartesian_descartes_planner.setConfiguration(descartes_config);
  cartesian_descartes_planner.solve(descartes_planner_response);

  if (!descartes_planner_response.status &&
      descartes_planner_response.status.value() !=
          tesseract_motion_planners::DescartesMotionPlannerStatusCategory::ErrorFoundValidSolutionInCollision)
  {
    ROS_ERROR("Failed to find seed motion plan for process: %s", descartes_planner_response.status.message().c_str());
    seed_process_plan.valid = false;
    return seed_process_plan;
  }

  std::size_t num_rows = 0;
  std::size_t idx = 0;

  // Get seed trajectory for from home
  num_rows = from_home_waypoints.size();
  seed_process_plan.from_start.trajectory =
      descartes_planner_response.joint_trajectory.trajectory.topRows(idx + num_rows).bottomRows(num_rows);
  seed_process_plan.from_start.joint_names = descartes_planner_response.joint_trajectory.joint_names;
  idx += num_rows;

  seed_process_plan.transition_from_end.resize(process_definition_.segments.size() - 1);
  for (size_t i = 0; i < process_definition_.segments.size(); ++i)
  {
    tesseract_process_planners::ProcessSegmentPlan seed_process_segment_plan;
    seed_process_segment_plan.valid = true;

    num_rows = process_definition_.segments[i].approach.size();
    seed_process_segment_plan.approach.trajectory =
        descartes_planner_response.joint_trajectory.trajectory.topRows(idx + num_rows).bottomRows(num_rows);
    seed_process_segment_plan.approach.joint_names = descartes_planner_response.joint_trajectory.joint_names;
    idx += num_rows;

    num_rows = process_definition_.segments[i].process.size();
    seed_process_segment_plan.process.trajectory =
        descartes_planner_response.joint_trajectory.trajectory.topRows(idx + num_rows).bottomRows(num_rows);
    seed_process_segment_plan.process.joint_names = descartes_planner_response.joint_trajectory.joint_names;
    idx += num_rows;

    num_rows = process_definition_.segments[i].departure.size();
    seed_process_segment_plan.departure.trajectory =
        descartes_planner_response.joint_trajectory.trajectory.topRows(idx + num_rows).bottomRows(num_rows);
    seed_process_segment_plan.departure.joint_names = descartes_planner_response.joint_trajectory.joint_names;
    idx += num_rows;

    seed_process_plan.segments.push_back(seed_process_segment_plan);

    if (i < process_definition_.segments.size() - 1)
    {
      num_rows = process_definition_.transitions[i].transition_from_end.size();
      seed_process_plan.transition_from_end[i].trajectory =
          descartes_planner_response.joint_trajectory.trajectory.topRows(idx + num_rows).bottomRows(num_rows);
      seed_process_plan.transition_from_end[i].joint_names = descartes_planner_response.joint_trajectory.joint_names;
      idx += num_rows;
    }
  }

  // Get seed trajectory for to home
  num_rows = to_home_waypoints.size();
  seed_process_plan.to_end.trajectory =
      descartes_planner_response.joint_trajectory.trajectory.topRows(idx + num_rows).bottomRows(num_rows);
  seed_process_plan.to_end.joint_names = descartes_planner_response.joint_trajectory.joint_names;

  return seed_process_plan;
}

void SandingProcessPlanner::computeFreeSpaceMotion(tesseract_motion_planners::PlannerResponse& planner_response,
                                                   const tesseract_motion_planners::JointWaypoint::Ptr& start_waypoint,
                                                   const tesseract_motion_planners::JointWaypoint::Ptr& end_waypoint)
{
  tesseract_kinematics::ForwardKinematics::ConstPtr robot_kin =
      config_.tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver(config_.robot);
  tesseract_kinematics::ForwardKinematics::ConstPtr full_kin =
      config_.tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver(config_.manipulator);
  int num_attempts = 3;
  ROS_INFO("Getting Seed from OMPL");
  tesseract_motion_planners::OMPLMotionPlanner ompl_planner;

  auto is_valid = [&full_kin, &robot_kin](const ompl::base::SpaceInformationPtr& si,
                                          const tesseract_motion_planners::OMPLPlannerConfig& config) {
    return std::make_shared<TWCStateValidityChecker>(si, config, full_kin, robot_kin);
  };

  auto sbl_planner_configurator = std::make_shared<tesseract_motion_planners::SBLConfigurator>();
  sbl_planner_configurator->range = 0.01;

  auto rrt_planner_configurator = std::make_shared<tesseract_motion_planners::RRTConnectConfigurator>();
  rrt_planner_configurator->range = 0.01;

  auto est_planner_configurator = std::make_shared<tesseract_motion_planners::ESTConfigurator>();
  est_planner_configurator->range = 0.01;
  est_planner_configurator->goal_bias = 0.5;

  std::vector<tesseract_motion_planners::OMPLPlannerConfigurator::ConstPtr> ompl_planners;
  ompl_planners.push_back(sbl_planner_configurator);
  ompl_planners.push_back(est_planner_configurator);
  ompl_planners.push_back(rrt_planner_configurator);
  ompl_planners.push_back(rrt_planner_configurator);

  auto ompl_config = std::make_shared<tesseract_motion_planners::OMPLPlannerFreespaceConfig>(
      config_.tesseract, config_.manipulator, ompl_planners);
  ompl_config->start_waypoint = start_waypoint;
  ompl_config->end_waypoint = end_waypoint;
  ompl_config->collision_safety_margin = FREESPACE_CONTACT_DISTANCE;
  ompl_config->planning_time = 5;
  ompl_config->simplify = false;
  ompl_config->optimize = true;
  ompl_config->collision_continuous = true;
  ompl_config->collision_check = true;
  ompl_config->weights.Ones(full_kin->numJoints());
  ompl_config->weights.cwiseInverse();
  ompl_config->max_solutions = 2;
  ompl_config->n_output_states = 20;
  ompl_config->longest_valid_segment_fraction = LONGEST_VALID_SEGMENT_FRACTION;
  ompl_config->svc_allocator = is_valid;

  for (int i = 0; i < num_attempts; ++i)
  {
    tesseract_motion_planners::PlannerResponse rrt_connect_planning_response;
    ompl_planner.setConfiguration(ompl_config);
    ompl_planner.solve(rrt_connect_planning_response);

    if (!rrt_connect_planning_response.status)
    {
      if (i != (num_attempts - 1))
        continue;

      planner_response = rrt_connect_planning_response;
      ROS_ERROR_STREAM("Failed to find OMPL seed for freespace motion plan within the timeout.");
      ROS_ERROR_STREAM(rrt_connect_planning_response.status.message().c_str());
      return;
    }

    auto ompl_seed = rrt_connect_planning_response.joint_trajectory.trajectory;

    // Setup freespace planner
    auto config = std::make_shared<tesseract_motion_planners::TrajOptPlannerFreespaceConfig>(
        config_.tesseract, config_.manipulator, config_.link, config_.tcp);
    config->target_waypoints = { start_waypoint, end_waypoint };
    config->num_steps = static_cast<int>(ompl_seed.rows());
    config->init_type = trajopt::InitInfo::GIVEN_TRAJ;
    config->optimizer = sco::ModelType::OSQP;
    config->longest_valid_segment_fraction = LONGEST_VALID_SEGMENT_FRACTION;
    config->params.max_iter = 10;
    config->velocity_coeff.Ones(full_kin->numJoints());
    config->acceleration_coeff.Ones(full_kin->numJoints());
    config->jerk_coeff.Ones(full_kin->numJoints());
    config->seed_trajectory = ompl_seed;
    config->collision_cost_config.enabled = false;
    config->collision_constraint_config.enabled = true;
    config->collision_constraint_config.type = trajopt::CollisionEvaluatorType::CAST_CONTINUOUS;
    config->collision_constraint_config.safety_margin = FREESPACE_CONTACT_DISTANCE;

    // Solve for motion plan
    ROS_INFO("Beginning Optimization with TrajOpt");
    tesseract_motion_planners::TrajOptMotionPlanner trajopt_planner;
    trajopt_planner.setConfiguration(config);
    trajopt_planner.solve(planner_response, PLANNING_VERBOSE);
    ROS_INFO("Finished Optimization with TrajOpt");

    // Check status
    if (!planner_response.status)
    {
      if (i != (num_attempts - 1))
        continue;

      ROS_WARN("Trajopt Failed: Returning OMPL Seed");
      planner_response = rrt_connect_planning_response;
    }
    else
    {
      return;
    }
  }
}
}  // namespace twc_motion_planning
