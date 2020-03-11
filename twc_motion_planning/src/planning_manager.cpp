
#include <twc_motion_planning/planning_manager.h>
#include <twc_motion_planning/sanding_process_planner.h>

#include <ros/topic.h>
#include <trajopt_utils/logging.hpp>

#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_kinematics/opw/opw_inv_kin.h>
#include <tesseract_motion_planners/core/waypoint.h>
#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_rosutils/conversions.h>

// For OMPL
#include <tesseract_motion_planners/ompl/continuous_motion_validator.h>
#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>
#include <tesseract_motion_planners/ompl/config/ompl_planner_freespace_config.h>
#include <tesseract_motion_planners/ompl/ompl_planner_configurator.h>

#include <memory>

namespace twc_motion_planning
{
using geometry_msgs::Pose;
using geometry_msgs::PoseArray;
using tesseract_environment::Environment;
using tesseract_kinematics::ForwardKinematics;
using tesseract_monitoring::EnvironmentMonitor;
using tesseract_motion_planners::CartesianWaypoint;
using tesseract_motion_planners::DescartesMotionPlannerStatusCategory;
using tesseract_motion_planners::JointWaypoint;
using tesseract_motion_planners::Waypoint;
using tesseract_process_planners::ProcessDefinition;
using tesseract_rosutils::toEigen;
using tesseract_rosutils::toJointTrajectory;
using tesseract_rosutils::toMsg;
using tesseract_rosutils::toProcessPlanPath;
using tesseract_rosutils::toProcessPlanSegement;
using tesseract_rosutils::toWaypoint;
using trajectory_msgs::JointTrajectory;

// ROS Interface topics
static const std::string NAME = "TWC";
static const std::string JOB_ACTION_NAME = "plan_job";
static const std::string JOINT_STATE_TOPIC = "joint_states";

// Link names common to multiple robots
static const std::string ROBOT_TOOL0_LINK_NAME = "robot_tool0";
static const std::string ROBOT_MANIPULATOR_NAME = "robot_only";
static const std::string ROBOT_IK_SOLVER_NAME = "OPWInvKin";
static const std::string PUSHCORP_MANIPULATOR_NAME = "pushcorp_only";
static const std::string POSITIONER_MANIPULATOR_NAME = "positioner_only";
static const std::string ROBOT_PUSHCORP_MANIPULATOR_NAME = "robot_only";    //"robot_pushcorp";
static const std::string ROBOT_POSITIONER_MANIPULATOR_NAME = "robot_only";  //"robot_positioner";

// Motion Planning Constants
static const double LONGEST_VALID_SEGMENT_FRACTION = 0.001;

PlanningManager::PlanningManager(ros::NodeHandle nh, tesseract::Tesseract::Ptr tesseract_ptr, std::string robot_id)
  : nh_(nh)
  , tesseract_(tesseract_ptr)
  , job_as_(nh_, JOB_ACTION_NAME, std::bind(&PlanningManager::jobExecuteCB, this, std::placeholders::_1), false)
  , robot_id_(robot_id)
{
  // Set the Trajopt logger to output debug information (useful when experiencing problems)
  //  util::gLogLevel = util::LogLevel::LevelFatal;

  // Set the OMPL Logger Level
  //  ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_ERROR);

  // First add opw inverse kinematics solvers to tesseract
  ForwardKinematics::Ptr robot_only =
      tesseract_->getFwdKinematicsManagerConst()->getFwdKinematicSolver(ROBOT_MANIPULATOR_NAME);
  opw_kinematics::Parameters<double> kin_params;
  kin_params = makeABBirb4600_60_205<double>();

  auto opw_inv_kin = std::make_shared<tesseract_kinematics::OPWInvKin>();
  opw_inv_kin->init(ROBOT_MANIPULATOR_NAME,
                    kin_params,
                    robot_only->getBaseLinkName(),
                    robot_only->getTipLinkName(),
                    robot_only->getJointNames(),
                    robot_only->getLinkNames(),
                    robot_only->getActiveLinkNames(),
                    robot_only->getLimits());

  tesseract_->getInvKinematicsManager()->addInvKinematicSolver(opw_inv_kin);

  env_monitor_.reset(new EnvironmentMonitor(tesseract_ptr, NAME, "", ""));
  env_monitor_->startPublishingEnvironment(tesseract_monitoring::EnvironmentMonitor::UPDATE_GEOMETRY);
  env_monitor_->startStateMonitor(JOINT_STATE_TOPIC);

  job_as_.registerPreemptCallback(boost::bind(&PlanningManager::jobPreemptCB, this));
  job_as_.start();

  // Process path publisher
  posearray_pub2_ = nh.advertise<geometry_msgs::PoseArray>("/twc_motion_planning/tool_path", 1, true);
  posearray_pub_ = nh.advertise<geometry_msgs::PoseArray>("/twc_motion_planning/process_path", 1, true);
}

void PlanningManager::jobExecuteCB(const twc_msgs::ProcessJobGoalConstPtr& goal)
{
  // Set Results to empty results message
  twc_msgs::ProcessJobResult job_result;

  // Get a pointer to the environment and kinematics object
  tesseract_environment::Environment::ConstPtr env = env_monitor_->getEnvironment();
  tesseract_kinematics::ForwardKinematics::ConstPtr kin =
      tesseract_->getFwdKinematicsManagerConst()->getFwdKinematicSolver(ROBOT_POSITIONER_MANIPULATOR_NAME);
  tesseract_kinematics::ForwardKinematics::ConstPtr robot_kin =
      tesseract_->getFwdKinematicsManagerConst()->getFwdKinematicSolver(ROBOT_MANIPULATOR_NAME);

  // Get current state transforms
  tesseract_common::TransformMap current_transforms = env->getCurrentState()->transforms;

  if (!kin)
  {
    job_result.success = false;
    job_result.error_message =
        "Failed to get manipulator for kinematic group '" + ROBOT_POSITIONER_MANIPULATOR_NAME + "'";
    job_as_.setSucceeded(job_result, job_result.error_message);
    return;
  }

  // Set the seed state
  Eigen::VectorXd seed_state = toEigen(goal->start_state, kin->getJointNames());
  Waypoint::Ptr start_state =
      std::make_shared<JointWaypoint>(toEigen(goal->start_state, kin->getJointNames()), kin->getJointNames());
  Waypoint::Ptr end_state =
      std::make_shared<JointWaypoint>(toEigen(goal->end_state, kin->getJointNames()), kin->getJointNames());

  Eigen::Isometry3d tcp = current_transforms["robot_tool0"].inverse() * current_transforms["st_tool0"];

  //  if (!getTCP(tcp, process.process_type))
  //  {
  //    job_result.success = false;
  //    job_result.error_message = "Failed to get tcp for process type '" + std::to_string(process.process_type.val) +
  //    "'"; setProcessServerEndState(job_result, goal); return;
  //  }

  // Create Process Planner
  ProcessPlannerConfig config;
  config.type = ProcessPlannerConfig::ConfigType::ROBOT_ONLY;
  config.start = std::static_pointer_cast<JointWaypoint>(start_state);
  config.end = std::static_pointer_cast<JointWaypoint>(start_state);
  config.tesseract = tesseract_;
  config.manipulator = ROBOT_POSITIONER_MANIPULATOR_NAME;
  config.link = ROBOT_TOOL0_LINK_NAME;
  config.robot = ROBOT_MANIPULATOR_NAME;
  config.ik_solver = ROBOT_IK_SOLVER_NAME;
  config.positioner = POSITIONER_MANIPULATOR_NAME;
  config.tcp = tcp;

  tf::poseMsgToEigen(goal->tool_offset, config.local_offset);

  SandingProcessPlanner process_planner(config);

  // The motion planner expects cartesion pose to be relative to world (root of environment)
  auto change_base = current_transforms.find(goal->header.frame_id);
  if (change_base == current_transforms.end())
  {
    job_result.success = false;
    job_result.error_message = "Failed to get tool path transform for frame id '" + goal->header.frame_id + "'";
    job_as_.setSucceeded(job_result, job_result.error_message);
    return;
  }

  // Set Process planner's tool path
  auto waypoint_paths = toWaypoint(goal->paths, change_base->second, env->getRootLinkName());

  // Set coefficients for all waypoints based on the process type.
  Eigen::VectorXd coeff = 10 * Eigen::VectorXd::Ones(6);
  coeff(5) = 0;
  for (auto& path : waypoint_paths)
    for (auto& waypoint : path)
      waypoint->setCoefficients(coeff);

  // Publish Process path
  PoseArray pose_array2;
  for (const auto& path : waypoint_paths)
    toMsg(pose_array2, path);
  pose_array2.header.frame_id = env->getRootLinkName();
  posearray_pub2_.publish(pose_array2);

  process_planner.setToolPath(waypoint_paths);
  const ProcessDefinition& process_def = process_planner.getProcessDefinition();

  // Publish Process path
  PoseArray pose_array;
  toMsg(pose_array, process_def);
  pose_array.header.frame_id = env->getRootLinkName();
  posearray_pub_.publish(pose_array);

  // Motion Plan
  tesseract_process_planners::ProcessPlan process_plan = process_planner.plan(seed_state);
  if (!process_plan.valid)
  {
    job_result.success = false;
    job_result.error_message = "Failed to compute process plan.";
    job_as_.setSucceeded(job_result, job_result.error_message);
    return;
  }

  // Convert Motion Plan to message
  tesseract_msgs::ProcessPlan process_plan_msg;

  double t = 0;
  process_plan_msg.from_start = toProcessPlanPath(process_plan.from_start);

  t = 0;
  for (auto& p : process_plan_msg.from_start.trajectory.points)
  {
    p.time_from_start.fromSec(t);
    t += 1.0;
  }

  process_plan_msg.segments.reserve(process_plan.segments.size());
  for (size_t i = 0; i < process_plan.segments.size(); ++i)
  {
    tesseract_msgs::ProcessPlanSegment segment_plan = toProcessPlanSegement(process_plan.segments[i]);

    t = 0;
    if (segment_plan.approach.trajectory.points.size() != 0)
      segment_plan.approach.trajectory.points[0].time_from_start.fromSec(t);

    for (size_t j = 1; j < segment_plan.approach.trajectory.points.size(); ++j)
    {
      CartesianWaypoint::Ptr wp1 = std::static_pointer_cast<CartesianWaypoint>(process_def.segments[i].approach[j - 1]);
      CartesianWaypoint::Ptr wp2 = std::static_pointer_cast<CartesianWaypoint>(process_def.segments[i].approach[j]);
      t += ((wp2->getPosition() - wp1->getPosition()).norm() / (1 / 1000.0));
      segment_plan.approach.trajectory.points[j].time_from_start.fromSec(t);
    }

    t = 0;
    if (segment_plan.approach.trajectory.points.size() != 0)
      segment_plan.process.trajectory.points[0].time_from_start.fromSec(t);

    for (size_t j = 1; j < segment_plan.process.trajectory.points.size(); ++j)
    {
      CartesianWaypoint::Ptr wp1 = std::static_pointer_cast<CartesianWaypoint>(process_def.segments[i].process[j - 1]);
      CartesianWaypoint::Ptr wp2 = std::static_pointer_cast<CartesianWaypoint>(process_def.segments[i].process[j]);
      t += ((wp2->getPosition() - wp1->getPosition()).norm() / (1 / 1000.0));
      segment_plan.process.trajectory.points[j].time_from_start.fromSec(t);
    }

    t = 0;
    if (segment_plan.approach.trajectory.points.size() != 0)
      segment_plan.departure.trajectory.points[0].time_from_start.fromSec(t);

    for (size_t j = 1; j < segment_plan.departure.trajectory.points.size(); ++j)
    {
      CartesianWaypoint::Ptr wp1 =
          std::static_pointer_cast<CartesianWaypoint>(process_def.segments[i].departure[j - 1]);
      CartesianWaypoint::Ptr wp2 = std::static_pointer_cast<CartesianWaypoint>(process_def.segments[i].departure[j]);
      t += ((wp2->getPosition() - wp1->getPosition()).norm() / (1 / 1000.0));
      segment_plan.departure.trajectory.points[j].time_from_start.fromSec(t);
    }

    process_plan_msg.segments.push_back(segment_plan);
  }

  process_plan_msg.transitions.resize(process_plan.segments.size() - 1);
  for (size_t i = 0; i < (process_plan.segments.size() - 1); ++i)
  {
    process_plan_msg.transitions[i].from_end = toProcessPlanPath(process_plan.transition_from_end[i]);
    t = 0;
    for (size_t j = 1; j < process_plan_msg.transitions[i].from_end.trajectory.points.size(); ++j)
    {
      t += 1.0;
      process_plan_msg.transitions[i].from_end.trajectory.points[j].time_from_start.fromSec(t);
    }
  }

  process_plan_msg.to_end = toProcessPlanPath(process_plan.to_end);
  t = 0;
  for (auto& p : process_plan_msg.to_end.trajectory.points)
  {
    p.time_from_start.fromSec(t);
    t += 0.5;
  }

  job_result.process_plans.push_back(process_plan_msg);
  job_result.success = true;
  job_result.error_message = "Successfully created job motion plan";
  job_as_.setSucceeded(job_result, job_result.error_message);
}

void PlanningManager::jobPreemptCB()
{
  ROS_WARN_STREAM(JOB_ACTION_NAME << ": Preempted");
  job_as_.setPreempted();
}

}  // namespace twc_motion_planning
