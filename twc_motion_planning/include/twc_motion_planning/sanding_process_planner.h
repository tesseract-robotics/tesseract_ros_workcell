#ifndef TWC_MOTION_PLANNING_SANDING_PROCESS_PLANNER_H
#define TWC_MOTION_PLANNING_SANDING_PROCESS_PLANNER_H

#include <tesseract_process_planners/process_planner.h>
#include <tesseract_motion_planners/descartes/descartes_motion_planner.h>
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_motion_planners/trajopt/config/trajopt_planner_default_config.h>
#include <twc_motion_planning/utils.h>

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <algorithm>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace twc_motion_planning
{
class SandingProcessPlanner : public tesseract_process_planners::ProcessPlanner
{
public:
  SandingProcessPlanner(ProcessPlannerConfig config);
  ~SandingProcessPlanner() override = default;

  void setToolPath(const std::vector<std::vector<tesseract_motion_planners::Waypoint::Ptr>>& tool_path) override;

  const tesseract_process_planners::ProcessDefinition& getProcessDefinition() const override;  // { return
                                                                                               // process_definition_; }

  tesseract_process_planners::ProcessPlan plan() override;

  tesseract_process_planners::ProcessPlan plan(const Eigen::VectorXd& seed) override;

private:
  tesseract_process_planners::ProcessPlan generateDenseSeedTrajectoryRobotOnly();

  std::pair<tesseract_process_planners::ProcessSegmentPlan, tesseract_motion_planners::PlannerResponse>
  generateSegmentSeedTrajectory(std::size_t segment_idx);

  std::pair<tesseract_process_planners::ProcessSegmentPlan, tesseract_motion_planners::PlannerResponse>
  generateSegmentSeedTrajectoryRobotOnly(std::size_t segment_idx);

  std::pair<tesseract_process_planners::ProcessSegmentPlan, tesseract_motion_planners::PlannerResponse>
  generateSegmentProcessTrajectory(std::size_t segment_idx);

  std::pair<tesseract_process_planners::ProcessSegmentPlan, tesseract_motion_planners::PlannerResponse>
  generateSegmentProcessTrajectoryRobotOnly(std::size_t segment_idx);

  void generateSegmentApproachTrajectoryRobotOnly(tesseract_process_planners::ProcessSegmentPlan& segment_plan,
                                                  std::size_t segment_idx);

  void generateSegmentDepartureTrajectoryRobotOnly(tesseract_process_planners::ProcessSegmentPlan& segment_plan,
                                                   std::size_t segment_idx);

  void generateSegmentTransitionTrajectoryRobotOnly(std::size_t segment_idx);

  /**
   * @brief Compute a free space motion plan with fixed start and end joint positions
   * @param planner_response The planner response
   * @param start_waypoint   The fixed start joint position
   * @param end_waypoint The fixed end joint position
   */
  void computeFreeSpaceMotion(tesseract_motion_planners::PlannerResponse& planner_response,
                              const tesseract_motion_planners::JointWaypoint::Ptr& start_waypoint,
                              const tesseract_motion_planners::JointWaypoint::Ptr& end_waypoint);

  ProcessPlannerConfig config_;
  tesseract_process_planners::ProcessDefinition process_definition_;
  tesseract_process_planners::ProcessPlan process_plan_;
  tesseract_process_planners::ProcessPlan seed_plan_;
};
}  // namespace twc_motion_planning

#endif  // SANDING_PROCESS_PLANNER_H
