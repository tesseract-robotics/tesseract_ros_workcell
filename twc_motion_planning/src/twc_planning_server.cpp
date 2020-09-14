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

namespace twc
{
TWCPlanningServer::TWCPlanningServer(const std::string& robot_description,
                                     std::string name,
                                     std::string discrete_plugin,
                                     std::string continuous_plugin)
  : tesseract_planning_server::TesseractPlanningServer (robot_description, name, discrete_plugin, continuous_plugin)
{
}

TWCPlanningServer::TWCPlanningServer(std::shared_ptr<tesseract::Tesseract> tesseract,
                                                 std::string name,
                                                 std::string discrete_plugin,
                                                 std::string continuous_plugin)
  : tesseract_planning_server::TesseractPlanningServer (tesseract, name, discrete_plugin, continuous_plugin)
{
}

}
