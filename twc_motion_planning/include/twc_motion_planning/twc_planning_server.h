/**
 * @file twc_planning_server.h
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
#ifndef TWC_PLANNING_SERVER_H
#define TWC_PLANNING_SERVER_H

#include <tesseract_planning_server/tesseract_planning_server.h>

namespace twc
{

class TWCPlanningServer : public tesseract_planning_server::TesseractPlanningServer
{
public:
  TWCPlanningServer(const std::string& robot_description,
                    std::string name,
                    std::string discrete_plugin = "",
                    std::string continuous_plugin = "");

  TWCPlanningServer(std::shared_ptr<tesseract::Tesseract> tesseract,
                    std::string name,
                    std::string discrete_plugin = "",
                    std::string continuous_plugin = "");

  void loadTWCPlannerProfilesLVS();
  void loadTWCDefaultProfilesFixed();
};

}

#endif // TWC_PLANNING_SERVER_H
