/**
 * @file twc_planning_server.cpp
 * @brief A TWC planning server with a default set of motion planners
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
#include <twc_motion_planning/twc_planning_server.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_default_plan_profile.h>
#include <tesseract_motion_planners/ompl/problem_generators/default_problem_generator.h>

namespace twc
{
TWCPlanningServer::TWCPlanningServer(const std::string& robot_description,
                                     std::string name,
                                     std::string discrete_plugin,
                                     std::string continuous_plugin)
  : tesseract_planning_server::TesseractPlanningServer (robot_description, name, discrete_plugin, continuous_plugin)
{
  loadTWCDefaultProfiles();
}

TWCPlanningServer::TWCPlanningServer(std::shared_ptr<tesseract::Tesseract> tesseract,
                                                 std::string name,
                                                 std::string discrete_plugin,
                                                 std::string continuous_plugin)
  : tesseract_planning_server::TesseractPlanningServer (tesseract, name, discrete_plugin, continuous_plugin)
{
  loadTWCDefaultProfiles();
}

void TWCPlanningServer::loadTWCDefaultProfiles()
{
  simple_plan_profiles_["FREESPACE"] = std::make_shared<tesseract_planning::SimplePlannerDefaultPlanProfile>(20, 20);
  simple_plan_profiles_["TRANSITION"] = std::make_shared<tesseract_planning::SimplePlannerDefaultPlanProfile>(10, 10);
  simple_plan_profiles_["RASTER"] = std::make_shared<tesseract_planning::SimplePlannerDefaultPlanProfile>(1, 1);

  auto p = std::make_shared<tesseract_planning::OMPLDefaultPlanProfile>();
  auto pp = std::make_shared<tesseract_planning::RRTConnectConfigurator>();
  pp->range = 0.1;
  p->planners.clear();
  p->planners.push_back(pp);
  p->planners.push_back(pp);

  ompl_plan_profiles_["FREESPACE"] = p;
  ompl_plan_profiles_["TRANSITION"] =p;
}

}
