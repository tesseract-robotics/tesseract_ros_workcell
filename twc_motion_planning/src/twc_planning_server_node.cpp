/**
 * @file twc_planning_server_node.cpp
 * @brief The Tesseract Workcell planning server node
 *
 * @author Levi Armstrong
 * @date August 18, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/environment.h>
#include <tesseract_planning_server/tesseract_planning_server.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_fixed_size_assign_plan_profile.h>
#include <tesseract_motion_planners/ompl/profile/ompl_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>
#include <tesseract_motion_planners/descartes/profile/descartes_default_plan_profile.h>
#include <tesseract_motion_planners/ompl/profile/ompl_default_plan_profile.h>
#include <descartes_light/edge_evaluators/compound_edge_evaluator.h>

#include <twc_motion_planning/utils.h>

using tesseract_planning_server::TesseractPlanningServer;
using tesseract_planning::ProfileDictionary;
using tesseract_planning::OMPLPlanProfile;
using tesseract_planning::OMPLDefaultPlanProfile;
using tesseract_planning::TrajOptCompositeProfile;
using tesseract_planning::TrajOptPlanProfile;
using tesseract_planning::TrajOptDefaultCompositeProfile;
using tesseract_planning::TrajOptDefaultPlanProfile;
using tesseract_planning::DescartesPlanProfile;
using tesseract_planning::DescartesDefaultPlanProfileD;

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description"; /**< Default ROS parameter for robot description */
const double LONGEST_VALID_SEGMENT_LENGTH = 0.01;
const double CONTACT_DISTANCE_THRESHOLD = 0.01;

static const std::string TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask";
static const std::string TRAJOPT_IFOPT_DEFAULT_NAMESPACE = "TrajOptIfoptMotionPlannerTask";
static const std::string OMPL_DEFAULT_NAMESPACE = "OMPLMotionPlannerTask";
static const std::string DESCARTES_DEFAULT_NAMESPACE = "DescartesMotionPlannerTask";
static const std::string SIMPLE_DEFAULT_NAMESPACE = "SimpleMotionPlannerTask";

std::shared_ptr<tesseract_planning::TrajOptDefaultCompositeProfile>
createTrajOptCompositeProfile(twc::ProfileType profile_type)
{
  auto trajopt_composite_profile = std::make_shared<tesseract_planning::TrajOptDefaultCompositeProfile>();
  trajopt_composite_profile->longest_valid_segment_length = LONGEST_VALID_SEGMENT_LENGTH;
  trajopt_composite_profile->collision_cost_config.enabled = true;
  trajopt_composite_profile->collision_cost_config.type = trajopt::CollisionEvaluatorType::DISCRETE_CONTINUOUS;
  trajopt_composite_profile->collision_cost_config.safety_margin = 2 * CONTACT_DISTANCE_THRESHOLD;
  trajopt_composite_profile->collision_cost_config.coeff = 10;
  trajopt_composite_profile->collision_constraint_config.enabled = true;
  trajopt_composite_profile->collision_constraint_config.type = trajopt::CollisionEvaluatorType::DISCRETE_CONTINUOUS;
  trajopt_composite_profile->collision_constraint_config.safety_margin = CONTACT_DISTANCE_THRESHOLD;
  trajopt_composite_profile->collision_constraint_config.safety_margin_buffer = 2 * CONTACT_DISTANCE_THRESHOLD;
  trajopt_composite_profile->collision_constraint_config.coeff = 1;
  trajopt_composite_profile->smooth_velocities = false;
  trajopt_composite_profile->smooth_accelerations = true;
  trajopt_composite_profile->smooth_jerks = true;

  Eigen::VectorXd joint_weights;
  if (profile_type == twc::ProfileType::ROBOT_ONLY)
  {
    joint_weights = Eigen::VectorXd::Ones(6);
    joint_weights[3] = 5;
  }
  else if (profile_type == twc::ProfileType::ROBOT_ON_RAIL)
  {
    joint_weights = Eigen::VectorXd::Ones(7);
    joint_weights[0] = 0.5;
  }
  else if (profile_type == twc::ProfileType::ROBOT_WITH_2AXIS_POSITIONER)
  {
    joint_weights = Eigen::VectorXd::Ones(8);
  }

  trajopt_composite_profile->velocity_coeff = 5 * joint_weights;
  trajopt_composite_profile->acceleration_coeff = 10 * joint_weights;
  trajopt_composite_profile->jerk_coeff = 15 * joint_weights;

  return trajopt_composite_profile;
}

std::shared_ptr<tesseract_planning::TrajOptDefaultPlanProfile> createTrajOptPlanProfile(twc::ProfileType /*profile_type*/)
{
  auto trajopt_plan_profile = std::make_shared<tesseract_planning::TrajOptDefaultPlanProfile>();

  // Tool z-axis free
  trajopt_plan_profile->cartesian_coeff = Eigen::VectorXd::Constant(6, 10);
  trajopt_plan_profile->cartesian_coeff(5) = 0;
  trajopt_plan_profile->term_type = trajopt::TermType::TT_COST;

  return trajopt_plan_profile;
}

std::shared_ptr<tesseract_planning::OMPLDefaultPlanProfile> createOMPLPlanProfile(twc::ProfileType profile_type)
{
  auto ompl_plan_profile = std::make_shared<tesseract_planning::OMPLDefaultPlanProfile>();
  ompl_plan_profile->collision_check_config.contact_manager_config.margin_data_override_type =
      tesseract_collision::CollisionMarginOverrideType::OVERRIDE_DEFAULT_MARGIN;
  ompl_plan_profile->collision_check_config.contact_manager_config.margin_data =
      tesseract_collision::CollisionMarginData(CONTACT_DISTANCE_THRESHOLD);
  ompl_plan_profile->collision_check_config.longest_valid_segment_length = LONGEST_VALID_SEGMENT_LENGTH;
  ompl_plan_profile->collision_check_config.type = tesseract_collision::CollisionEvaluatorType::DISCRETE;
  ompl_plan_profile->planning_time = 120;

  auto rrtc1 = std::make_shared<tesseract_planning::RRTConnectConfigurator>();
  rrtc1->range = 0.1;

  auto rrtc2 = std::make_shared<tesseract_planning::RRTConnectConfigurator>();
  rrtc2->range = 0.2;

  auto est1 = std::make_shared<tesseract_planning::ESTConfigurator>();
  est1->range = 0.1;
  est1->goal_bias = 0.5;

  auto est2 = std::make_shared<tesseract_planning::ESTConfigurator>();
  est2->range = 0.1;
  est2->goal_bias = 0.25;

  auto est3 = std::make_shared<tesseract_planning::ESTConfigurator>();
  est3->range = 0.1;
  est3->goal_bias = 0.75;

  ompl_plan_profile->planners.clear();
  ompl_plan_profile->planners.push_back(rrtc1);
  ompl_plan_profile->planners.push_back(rrtc1);
  ompl_plan_profile->planners.push_back(rrtc1);
  ompl_plan_profile->planners.push_back(rrtc2);
  ompl_plan_profile->planners.push_back(rrtc2);
  ompl_plan_profile->planners.push_back(rrtc2);
  ompl_plan_profile->planners.push_back(est1);
  ompl_plan_profile->planners.push_back(est2);
  ompl_plan_profile->planners.push_back(est3);
  ompl_plan_profile->planners.push_back(est1);
  ompl_plan_profile->planners.push_back(est2);
  ompl_plan_profile->planners.push_back(est3);

  Eigen::VectorXd joint_weights;
  if (profile_type == twc::ProfileType::ROBOT_ONLY)
  {
    joint_weights = Eigen::VectorXd::Ones(6);
    joint_weights[3] = 5;
  }
  else if (profile_type == twc::ProfileType::ROBOT_ON_RAIL)
  {
    joint_weights = Eigen::VectorXd::Ones(7);
    joint_weights[0] = 0.5;
  }
  else if (profile_type == twc::ProfileType::ROBOT_WITH_2AXIS_POSITIONER)
  {
    joint_weights = Eigen::VectorXd::Ones(8);
  }

  ompl_plan_profile->state_sampler_allocator = [joint_weights](const ompl::base::StateSpace* space,
                                                  const tesseract_planning::OMPLProblem& prob) {
    const auto& limits = prob.manip->getLimits().joint_limits;
    return tesseract_planning::allocWeightedRealVectorStateSampler(space, joint_weights, limits);
  };

  return ompl_plan_profile;
}

std::shared_ptr<tesseract_planning::DescartesDefaultPlanProfile<float>>
createDescartesPlanProfile(twc::ProfileType profile_type)
{
  auto descartes_plan_profile = std::make_shared<tesseract_planning::DescartesDefaultPlanProfile<float>>();
  descartes_plan_profile->allow_collision = true;
  descartes_plan_profile->vertex_collision_check_config.contact_manager_config.margin_data_override_type =
      tesseract_collision::CollisionMarginOverrideType::OVERRIDE_DEFAULT_MARGIN;
  descartes_plan_profile->vertex_collision_check_config.contact_manager_config.margin_data =
      tesseract_collision::CollisionMarginData(1.5 * CONTACT_DISTANCE_THRESHOLD);
  descartes_plan_profile->num_threads = static_cast<int>(std::thread::hardware_concurrency());

  descartes_plan_profile->target_pose_sampler = [](const Eigen::Isometry3d& pose){return tesseract_planning::sampleToolZAxis(pose, M_PI_4); };

  // Add Vertex Evaluator
  descartes_plan_profile->vertex_evaluator = [](const tesseract_planning::DescartesProblem<float>& prob) {
    return std::make_shared<twc::DescartesStateValidator>(prob.manip, "robot_base_link", "robot_tool0");
  };

  Eigen::VectorXd joint_weights;
  if (profile_type == twc::ProfileType::ROBOT_ONLY)
  {
    joint_weights = Eigen::VectorXd::Ones(6);
    joint_weights[3] = 5;
  }
  else if (profile_type == twc::ProfileType::ROBOT_ON_RAIL)
  {
    joint_weights = Eigen::VectorXd::Ones(7);
    joint_weights[0] = 0.5;
  }
  else if (profile_type == twc::ProfileType::ROBOT_WITH_2AXIS_POSITIONER)
  {
    joint_weights = Eigen::VectorXd::Ones(8);
  }

  descartes_plan_profile->edge_evaluator = [joint_weights](const tesseract_planning::DescartesProblem<float>& prob) {
        auto e = std::make_shared<descartes_light::CompoundEdgeEvaluator<float>>();
        e->evaluators.push_back(std::make_shared<twc::RobotConfigEdgeEvaluator<float>>(prob.manip, "robot_base_link", "robot_tool0"));
        e->evaluators.push_back(std::make_shared<twc::WeightedEuclideanDistanceEdgeEvaluator<float>>(joint_weights));
        return e;
      };

  return descartes_plan_profile;
}

void loadTWCDefaultProfiles(TesseractPlanningServer& planning_server)
{
  ProfileDictionary& dict = planning_server.getProfileDictionary();

  { // Trajopt Composite Profiles
    auto p = createTrajOptCompositeProfile(twc::ProfileType::ROBOT_ONLY);
    dict.addProfile<TrajOptCompositeProfile>(TRAJOPT_DEFAULT_NAMESPACE, "FREESPACE_ROBOT", p);
    dict.addProfile<TrajOptCompositeProfile>(TRAJOPT_DEFAULT_NAMESPACE, "TRANSITION_ROBOT", p);
    dict.addProfile<TrajOptCompositeProfile>(TRAJOPT_DEFAULT_NAMESPACE, "RASTER_ROBOT", p);

    p = createTrajOptCompositeProfile(twc::ProfileType::ROBOT_ON_RAIL);
    dict.addProfile<TrajOptCompositeProfile>(TRAJOPT_DEFAULT_NAMESPACE, "FREESPACE_RAIL", p);
    dict.addProfile<TrajOptCompositeProfile>(TRAJOPT_DEFAULT_NAMESPACE, "TRANSITION_RAIL", p);
    dict.addProfile<TrajOptCompositeProfile>(TRAJOPT_DEFAULT_NAMESPACE, "RASTER_RAIL", p);

    p = createTrajOptCompositeProfile(twc::ProfileType::ROBOT_WITH_2AXIS_POSITIONER);
    dict.addProfile<TrajOptCompositeProfile>(TRAJOPT_DEFAULT_NAMESPACE, "FREESPACE_POSITIONER", p);
    dict.addProfile<TrajOptCompositeProfile>(TRAJOPT_DEFAULT_NAMESPACE, "TRANSITION_POSITIONER", p);
    dict.addProfile<TrajOptCompositeProfile>(TRAJOPT_DEFAULT_NAMESPACE, "RASTER_POSITIONER", p);
  }

  { // Trajopt Plan Profiles
    auto p = createTrajOptPlanProfile(twc::ProfileType::ROBOT_ONLY);
    dict.addProfile<TrajOptPlanProfile>(TRAJOPT_DEFAULT_NAMESPACE, "FREESPACE_ROBOT", p);
    dict.addProfile<TrajOptPlanProfile>(TRAJOPT_DEFAULT_NAMESPACE, "TRANSITION_ROBOT", p);
    dict.addProfile<TrajOptPlanProfile>(TRAJOPT_DEFAULT_NAMESPACE, "RASTER_ROBOT", p);

    p = createTrajOptPlanProfile(twc::ProfileType::ROBOT_ON_RAIL);
    dict.addProfile<TrajOptPlanProfile>(TRAJOPT_DEFAULT_NAMESPACE, "FREESPACE_RAIL", p);
    dict.addProfile<TrajOptPlanProfile>(TRAJOPT_DEFAULT_NAMESPACE, "TRANSITION_RAIL", p);
    dict.addProfile<TrajOptPlanProfile>(TRAJOPT_DEFAULT_NAMESPACE, "RASTER_RAIL", p);

    p = createTrajOptPlanProfile(twc::ProfileType::ROBOT_WITH_2AXIS_POSITIONER);
    dict.addProfile<TrajOptPlanProfile>(TRAJOPT_DEFAULT_NAMESPACE, "FREESPACE_POSITIONER", p);
    dict.addProfile<TrajOptPlanProfile>(TRAJOPT_DEFAULT_NAMESPACE, "TRANSITION_POSITIONER", p);
    dict.addProfile<TrajOptPlanProfile>(TRAJOPT_DEFAULT_NAMESPACE, "RASTER_POSITIONER", p);
  }

  { // OMPL Plan Profiles
    auto p = createOMPLPlanProfile(twc::ProfileType::ROBOT_ONLY);
    dict.addProfile<OMPLPlanProfile>(OMPL_DEFAULT_NAMESPACE, "FREESPACE_ROBOT", p);
    dict.addProfile<OMPLPlanProfile>(OMPL_DEFAULT_NAMESPACE, "TRANSITION_ROBOT", p);
    dict.addProfile<OMPLPlanProfile>(OMPL_DEFAULT_NAMESPACE, "RASTER_ROBOT", p);

    p = createOMPLPlanProfile(twc::ProfileType::ROBOT_ON_RAIL);
    dict.addProfile<OMPLPlanProfile>(OMPL_DEFAULT_NAMESPACE, "FREESPACE_RAIL", p);
    dict.addProfile<OMPLPlanProfile>(OMPL_DEFAULT_NAMESPACE, "TRANSITION_RAIL", p);
    dict.addProfile<OMPLPlanProfile>(OMPL_DEFAULT_NAMESPACE, "RASTER_RAIL", p);

    p = createOMPLPlanProfile(twc::ProfileType::ROBOT_WITH_2AXIS_POSITIONER);
    dict.addProfile<OMPLPlanProfile>(OMPL_DEFAULT_NAMESPACE, "FREESPACE_POSITIONER", p);
    dict.addProfile<OMPLPlanProfile>(OMPL_DEFAULT_NAMESPACE, "TRANSITION_POSITIONER", p);
    dict.addProfile<OMPLPlanProfile>(OMPL_DEFAULT_NAMESPACE, "RASTER_POSITIONER", p);
  }

  { // Descartes Plan Profiles
    auto p = createDescartesPlanProfile(twc::ProfileType::ROBOT_ONLY);
    dict.addProfile<DescartesPlanProfile<float>>(DESCARTES_DEFAULT_NAMESPACE, "FREESPACE_ROBOT", p);
    dict.addProfile<DescartesPlanProfile<float>>(DESCARTES_DEFAULT_NAMESPACE, "TRANSITION_ROBOT", p);
    dict.addProfile<DescartesPlanProfile<float>>(DESCARTES_DEFAULT_NAMESPACE, "RASTER_ROBOT", p);

    p = createDescartesPlanProfile(twc::ProfileType::ROBOT_ON_RAIL);
    dict.addProfile<DescartesPlanProfile<float>>(DESCARTES_DEFAULT_NAMESPACE, "FREESPACE_RAIL", p);
    dict.addProfile<DescartesPlanProfile<float>>(DESCARTES_DEFAULT_NAMESPACE, "TRANSITION_RAIL", p);
    dict.addProfile<DescartesPlanProfile<float>>(DESCARTES_DEFAULT_NAMESPACE, "RASTER_RAIL", p);

    p = createDescartesPlanProfile(twc::ProfileType::ROBOT_WITH_2AXIS_POSITIONER);
    dict.addProfile<DescartesPlanProfile<float>>(DESCARTES_DEFAULT_NAMESPACE, "FREESPACE_POSITIONER", p);
    dict.addProfile<DescartesPlanProfile<float>>(DESCARTES_DEFAULT_NAMESPACE, "TRANSITION_POSITIONER", p);
    dict.addProfile<DescartesPlanProfile<float>>(DESCARTES_DEFAULT_NAMESPACE, "RASTER_POSITIONER", p);
  }
}

static std::shared_ptr<TesseractPlanningServer> planning_server;

void updateCacheCallback(const ros::TimerEvent&) { planning_server->getEnvironmentCache().refreshCache(); }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "twc_planning_server");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  std::string robot_description;
  std::string monitor_namespace;
  std::string monitored_namespace;
  std::string task_composer_config;
  std::string input_key;
  std::string output_key;
  bool publish_environment{ false };
  int cache_size{ 5 };
  double cache_refresh_rate{ 0.1 };

  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);

  if (!pnh.getParam("monitor_namespace", monitor_namespace))
  {
    ROS_ERROR("Missing required parameter monitor_namespace!");
    return 1;
  }

  if (!pnh.getParam("input_key", input_key))
  {
    ROS_ERROR("Missing required parameter input_key!");
    return 1;
  }

  if (!pnh.getParam("output_key", output_key))
  {
    ROS_ERROR("Missing required parameter output_key!");
    return 1;
  }

  pnh.param<std::string>("monitored_namespace", monitored_namespace, "");
  pnh.param<std::string>("robot_description", robot_description, ROBOT_DESCRIPTION_PARAM);
  pnh.param<bool>("publish_environment", publish_environment, publish_environment);
  pnh.param<int>("cache_size", cache_size, cache_size);
  pnh.param<double>("cache_refresh_rate", cache_refresh_rate, cache_refresh_rate);
  pnh.param<std::string>("task_composer_config", task_composer_config, task_composer_config);

  planning_server = std::make_shared<TesseractPlanningServer>(robot_description,
                                                              input_key,
                                                              output_key,
                                                              monitor_namespace);
  loadTWCDefaultProfiles(*planning_server);

  planning_server->getEnvironmentCache().setCacheSize(cache_size);

  if (publish_environment)
    planning_server->getEnvironmentMonitor().startPublishingEnvironment();

  if (!monitored_namespace.empty())
    planning_server->getEnvironmentMonitor().startMonitoringEnvironment(monitored_namespace);

  if (!task_composer_config.empty())
  {
    tesseract_common::fs::path config(task_composer_config);
    planning_server->getTaskComposerServer().loadConfig(config);
  }

  ros::Timer update_cache = nh.createTimer(ros::Duration(cache_refresh_rate), updateCacheCallback);

  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::waitForShutdown();

  return 0;
}
