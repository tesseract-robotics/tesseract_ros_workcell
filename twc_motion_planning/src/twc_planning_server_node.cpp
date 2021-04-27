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

#include <tesseract_environment/core/environment.h>
#include <tesseract_planning_server/tesseract_planning_server.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_fixed_size_assign_plan_profile.h>
#include <tesseract_motion_planners/ompl/profile/ompl_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>
#include <tesseract_motion_planners/descartes/profile/descartes_default_plan_profile.h>

#include <tesseract_process_managers/taskflow_generators/descartes_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/trajopt_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/freespace_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/raster_global_taskflow.h>
#include <tesseract_process_managers/taskflow_generators/raster_taskflow.h>

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
using tesseract_planning::TaskflowGenerator;
using tesseract_planning::DescartesTaskflow;
using tesseract_planning::DescartesTaskflowParams;
using tesseract_planning::TrajOptTaskflow;
using tesseract_planning::TrajOptTaskflowParams;
using tesseract_planning::FreespaceTaskflow;
using tesseract_planning::FreespaceTaskflowParams;
using tesseract_planning::RasterGlobalTaskflow;
using tesseract_planning::RasterTaskflow;

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description"; /**< Default ROS parameter for robot description */

void loadTWCDefaultProfiles(TesseractPlanningServer& planning_server)
{
  ProfileDictionary::Ptr dict = planning_server.getProcessPlanningServer().getProfiles();

  auto p = std::make_shared<tesseract_planning::OMPLDefaultPlanProfile>();
  p->collision_check_config.longest_valid_segment_length = 0.1;

  auto pp = std::make_shared<tesseract_planning::RRTConnectConfigurator>();
  pp->range = 0.1;
  p->planners.clear();
  p->planners.push_back(pp);
  p->planners.push_back(pp);

  dict->addProfile<OMPLPlanProfile>("FREESPACE", p);
  dict->addProfile<OMPLPlanProfile>("TRANSITION", p);

  auto trajopt_composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();
  trajopt_composite_profile->longest_valid_segment_length = 0.1;

  dict->addProfile<TrajOptCompositeProfile>("FREESPACE", trajopt_composite_profile);
  dict->addProfile<TrajOptCompositeProfile>("TRANSITION", trajopt_composite_profile);
  dict->addProfile<TrajOptCompositeProfile>("RASTER", trajopt_composite_profile);

  auto trajopt_plan_profile = std::make_shared<TrajOptDefaultPlanProfile>();
  trajopt_plan_profile->cartesian_coeff.resize(6);
  trajopt_plan_profile->cartesian_coeff << 5, 5, 5, 5, 5, 0;

  dict->addProfile<TrajOptPlanProfile>("FREESPACE", trajopt_plan_profile);
  dict->addProfile<TrajOptPlanProfile>("TRANSITION", trajopt_plan_profile);
  dict->addProfile<TrajOptPlanProfile>("RASTER", trajopt_plan_profile);

  auto descartes_plan_profile = std::make_shared<DescartesDefaultPlanProfileD>();
  descartes_plan_profile->target_pose_sampler = [](const Eigen::Isometry3d& pose){return tesseract_planning::sampleToolZAxis(pose, M_PI_4); };
  dict->addProfile<DescartesPlanProfile<double>>("FREESPACE", descartes_plan_profile);
  dict->addProfile<DescartesPlanProfile<double>>("TRANSITION", descartes_plan_profile);
  dict->addProfile<DescartesPlanProfile<double>>("RASTER", descartes_plan_profile);
}

TaskflowGenerator::UPtr createRasterDebugGenerator()
{
  tesseract_planning::DescartesTaskflowParams params;
  params.enable_post_contact_discrete_check = false;
  params.enable_post_contact_continuous_check = false;
  params.enable_time_parameterization = true;

  TaskflowGenerator::UPtr freespace_task = std::make_unique<DescartesTaskflow>(params);
  TaskflowGenerator::UPtr transition_task = std::make_unique<DescartesTaskflow>(params);
  TaskflowGenerator::UPtr raster_task = std::make_unique<DescartesTaskflow>(params);

  return std::make_unique<RasterTaskflow>(std::move(freespace_task), std::move(transition_task), std::move(raster_task));
}

TaskflowGenerator::UPtr createRasterGlobalDebugGenerator()
{
  tesseract_planning::DescartesTaskflowParams params;
  params.enable_post_contact_discrete_check = false;
  params.enable_post_contact_continuous_check = false;
  params.enable_time_parameterization = true;

  return std::make_unique<tesseract_planning::DescartesTaskflow>(params);
}

TaskflowGenerator::UPtr createRasterGlobalNoPostCheckGenerator()
{
  tesseract_planning::DescartesTaskflowParams global_params;
  global_params.enable_post_contact_discrete_check = false;
  global_params.enable_post_contact_continuous_check = false;
  global_params.enable_time_parameterization = false;
  TaskflowGenerator::UPtr global_task = std::make_unique<DescartesTaskflow>(global_params);

  tesseract_planning::FreespaceTaskflowParams freespace_params;
  freespace_params.enable_post_contact_discrete_check = false;
  freespace_params.enable_post_contact_continuous_check = false;
  freespace_params.type = tesseract_planning::FreespaceTaskflowType::TRAJOPT_FIRST;
  TaskflowGenerator::UPtr freespace_task = std::make_unique<tesseract_planning::FreespaceTaskflow>(freespace_params);

  tesseract_planning::FreespaceTaskflowParams transition_params;
  transition_params.enable_post_contact_discrete_check = false;
  transition_params.enable_post_contact_continuous_check = false;
  transition_params.type = tesseract_planning::FreespaceTaskflowType::TRAJOPT_FIRST;
  TaskflowGenerator::UPtr transition_task = std::make_unique<tesseract_planning::FreespaceTaskflow>(transition_params);

  tesseract_planning::TrajOptTaskflowParams raster_params;
  raster_params.enable_post_contact_discrete_check = false;
  raster_params.enable_post_contact_continuous_check = false;
  TaskflowGenerator::UPtr raster_task = std::make_unique<tesseract_planning::TrajOptTaskflow>(raster_params);

  return std::make_unique<tesseract_planning::RasterGlobalTaskflow>(
      std::move(global_task), std::move(freespace_task), std::move(transition_task), std::move(raster_task));
}

TaskflowGenerator::UPtr createRasterTrajOptGenerator()
{
  // Create Freespace and Transition Taskflows
  FreespaceTaskflowParams fparams;
  TaskflowGenerator::UPtr freespace_task = std::make_unique<FreespaceTaskflow>(fparams);
  TaskflowGenerator::UPtr transition_task = std::make_unique<FreespaceTaskflow>(fparams);

  // Create Raster Taskflow
  tesseract_planning::TrajOptTaskflowParams raster_params;
  TaskflowGenerator::UPtr raster_task = std::make_unique<tesseract_planning::TrajOptTaskflow>(raster_params);

  return std::make_unique<RasterTaskflow>(
      std::move(freespace_task), std::move(transition_task), std::move(raster_task));
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "twc_planning_server");
  ros::NodeHandle pnh("~");
  std::string robot_description;
  std::string discrete_plugin;
  std::string continuous_plugin;
  std::string monitor_namespace;
  std::string monitored_namespace;
  bool publish_environment{ false };
  int threads = static_cast<int>(std::thread::hardware_concurrency());

  if (!pnh.getParam("monitor_namespace", monitor_namespace))
  {
    ROS_ERROR("Missing required parameter monitor_namespace!");
    return 1;
  }

  pnh.param<std::string>("monitored_namespace", monitored_namespace, "");
  pnh.param<std::string>("robot_description", robot_description, ROBOT_DESCRIPTION_PARAM);
  pnh.param<std::string>("discrete_plugin", discrete_plugin, "");
  pnh.param<std::string>("continuous_plugin", continuous_plugin, "");
  pnh.param<bool>("publish_environment", publish_environment, publish_environment);
  pnh.param<int>("threads", threads, threads);

  TesseractPlanningServer planning_server(robot_description, monitor_namespace, static_cast<std::size_t>(threads), discrete_plugin, continuous_plugin);
  loadTWCDefaultProfiles(planning_server);

  planning_server.getProcessPlanningServer().registerProcessPlanner("RasterTrajOpt", createRasterTrajOptGenerator());
  planning_server.getProcessPlanningServer().registerProcessPlanner("RasterDebug", createRasterDebugGenerator());
  planning_server.getProcessPlanningServer().registerProcessPlanner("RasterGDebug", createRasterGlobalDebugGenerator());
  planning_server.getProcessPlanningServer().registerProcessPlanner("RasterGNoPostCheckDebug", createRasterGlobalNoPostCheckGenerator());

  if (publish_environment)
    planning_server.getEnvironmentMonitor().startPublishingEnvironment();

  if (!monitored_namespace.empty())
    planning_server.getEnvironmentMonitor().startMonitoringEnvironment(monitored_namespace);

  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::waitForShutdown();

  return 0;
}
