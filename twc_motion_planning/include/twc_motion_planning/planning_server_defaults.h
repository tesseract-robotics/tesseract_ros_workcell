#ifndef TWC_PLANNING_SERVER_DEFAULTS_H
#define TWC_PLANNING_SERVER_DEFAULTS_H

#include <tesseract_process_managers/core/process_planning_server.h>
namespace twc
{
void loadTWCProfiles(tesseract_planning::ProcessPlanningServer& planning_server);

void registerTWCProcessPlanners(tesseract_planning::ProcessPlanningServer& planning_server);
}
#endif // TWC_PLANNING_SERVER_DEFAULTS_H
