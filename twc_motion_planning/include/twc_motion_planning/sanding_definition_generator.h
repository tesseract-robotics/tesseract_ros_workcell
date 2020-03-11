#ifndef TESSERACT_PROCESS_PLANNERS_SANDING_DEFINITION_GENERATOR_H
#define TESSERACT_PROCESS_PLANNERS_SANDING_DEFINITION_GENERATOR_H

#include <twc_motion_planning/process_definition_generator.h>

namespace tesseract_process_planners
{
class SandingDefinitionGenerator : public ProcessDefinitionGenerator
{
public:
  SandingDefinitionGenerator() = default;
  ~SandingDefinitionGenerator() override = default;

  void setLocalOffset(const Eigen::Isometry3d& local_offset);
  const Eigen::Isometry3d& getLocalOffset() const;

  void setWorldOffset(const Eigen::Isometry3d& world_offset);
  const Eigen::Isometry3d& getWorldOffset() const;

  void setStartWaypoint(const tesseract_motion_planners::Waypoint::ConstPtr& start_waypoint);
  const tesseract_motion_planners::Waypoint::ConstPtr& getStartWaypoint() const;

  void setEndWaypoint(const tesseract_motion_planners::Waypoint::ConstPtr& end_waypoint);
  const tesseract_motion_planners::Waypoint::ConstPtr& getEndWaypoint() const;

  ProcessDefinition generate(tesseract_motion_planners::Waypoint::ConstPtr start,
                             tesseract_motion_planners::Waypoint::ConstPtr end,
                             const ToolPaths& tool_paths) const override;

  ProcessDefinition generate(const ToolPaths& tool_paths) const override;

private:
  Eigen::Isometry3d local_offset_{ Eigen::Isometry3d::Identity() };
  Eigen::Isometry3d world_offset_{ Eigen::Isometry3d::Identity() };
  tesseract_motion_planners::Waypoint::ConstPtr start_waypoint_;
  tesseract_motion_planners::Waypoint::ConstPtr end_waypoint_;
};
}  // namespace tesseract_process_planners
#endif  // TESSERACT_PROCESS_PLANNERS_SANDING_DEFINITION_GENERATOR_H
