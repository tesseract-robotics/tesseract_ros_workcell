#include <twc_motion_planning/sanding_definition_generator.h>
#include <tesseract_process_planners/generators/axial_approach_generator.h>
#include <tesseract_process_planners/generators/subsample_process_generator.h>
#include <tesseract_process_planners/generators/passthrough_transition_generator.h>
#include <tesseract_process_planners/generators/axial_departure_generator.h>

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_process_planners
{
void SandingDefinitionGenerator::setLocalOffset(const Eigen::Isometry3d& local_offset) { local_offset_ = local_offset; }

const Eigen::Isometry3d& SandingDefinitionGenerator::getLocalOffset() const { return local_offset_; }

void SandingDefinitionGenerator::setWorldOffset(const Eigen::Isometry3d& world_offset) { world_offset_ = world_offset; }

const Eigen::Isometry3d& SandingDefinitionGenerator::getWorldOffset() const { return world_offset_; }

void SandingDefinitionGenerator::setStartWaypoint(const tesseract_motion_planners::Waypoint::ConstPtr& start_waypoint)
{
  start_waypoint_ = start_waypoint;
}
const tesseract_motion_planners::Waypoint::ConstPtr& SandingDefinitionGenerator::getStartWaypoint() const
{
  return start_waypoint_;
}

void SandingDefinitionGenerator::setEndWaypoint(const tesseract_motion_planners::Waypoint::ConstPtr& end_waypoint)
{
  end_waypoint_ = end_waypoint;
}

const tesseract_motion_planners::Waypoint::ConstPtr& SandingDefinitionGenerator::getEndWaypoint() const
{
  return end_waypoint_;
}

ProcessDefinition SandingDefinitionGenerator::generate(tesseract_motion_planners::Waypoint::ConstPtr start,
                                                       tesseract_motion_planners::Waypoint::ConstPtr end,
                                                       const ToolPaths& tool_paths) const
{
  auto transition_generator = std::make_shared<tesseract_process_planners::PassthroughTransitionGenerator>();

  auto process_config = tesseract_process_planners::ProcessDefinitionConfig();
  process_config.start = start;
  process_config.end = end;
  process_config.tool_paths = tool_paths;
  process_config.transition_generator = std::vector<tesseract_process_planners::ProcessTransitionGenerator::ConstPtr>(
      tool_paths.size() - 1, transition_generator);
  process_config.local_offset_direction = local_offset_;
  process_config.world_offset_direction = world_offset_;

  Eigen::Isometry3d approach_direction = Eigen::Isometry3d::Identity();
  approach_direction.translation()(2) = -0.05;

  Eigen::Isometry3d departure_direction = Eigen::Isometry3d::Identity();
  departure_direction.translation()(2) = -0.05;

  tesseract_process_planners::ProcessSegmentDefinitionConfig segment_config;

  segment_config.approach = std::make_shared<tesseract_process_planners::AxialApproachGenerator>(approach_direction, 5);
  segment_config.departure =
      std::make_shared<tesseract_process_planners::AxialDepartureGenerator>(departure_direction, 5);
  segment_config.process = std::make_shared<tesseract_process_planners::SubsampleProcessGenerator>(5);

  std::vector<tesseract_process_planners::ProcessSegmentDefinitionConfig> segment_configs;
  segment_configs.reserve(process_config.tool_paths.size());
  for (std::size_t i = 0; i < process_config.tool_paths.size(); i++)
    segment_configs.push_back(segment_config);

  return tesseract_process_planners::generateProcessDefinition(process_config, segment_configs);
}

ProcessDefinition SandingDefinitionGenerator::generate(const ToolPaths& tool_paths) const
{
  return generate(start_waypoint_, end_waypoint_, tool_paths);
}

}  // namespace tesseract_process_planners
