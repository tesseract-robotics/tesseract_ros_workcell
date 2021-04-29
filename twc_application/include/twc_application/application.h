#ifndef TWC_APPLICATION_APPLICATION_H
#define TWC_APPLICATION_APPLICATION_H

#include <tesseract_environment/core/environment.h>
#include <tesseract_visualization/visualization.h>
#include <tesseract_visualization/markers/toolpath_marker.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_command_language/core/instruction.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_common/types.h>

namespace twc
{

class Application
{
public:
  Application(tesseract_environment::Environment::Ptr env,
              tesseract_visualization::Visualization::Ptr plotter)
    : env_(std::move(env))
    , plotter_(std::move(plotter))
  {
  }
  virtual ~Application() = default;

  virtual void run() = 0;

  virtual void plotToolpath(const tesseract_planning::Instruction& program)
  {
    if (plotter_ != nullptr && env_ != nullptr)
    {
      tesseract_common::Toolpath tp = tesseract_planning::toToolpath(program, env_);
      plotter_->plotMarker(tesseract_visualization::ToolpathMarker(tp));
      plotter_->waitForInput("Preview Toolpath. Hit enter to continue!");
    }
  }

  virtual void plotTrajectory(const tesseract_planning::Instruction& program)
  {
    if (plotter_ != nullptr && env_ != nullptr)
    {
      const auto& ci = program.as<tesseract_planning::CompositeInstruction>();
      long num_wp = tesseract_planning::getMoveInstructionCount(ci);

      tesseract_common::Toolpath tp = tesseract_planning::toToolpath(program, env_);
      plotter_->plotMarker(tesseract_visualization::ToolpathMarker(tp));
      plotter_->waitForInput("Preview Trajectory Toolpath. Hit enter to continue!");

      plotter_->plotTrajectory(tesseract_planning::toJointTrajectory(ci), env_->getStateSolver());
      plotter_->waitForInput("Preview Trajectory. Hit enter to continue!");
    }
  }

protected:
  tesseract_environment::Environment::Ptr env_;
  tesseract_visualization::Visualization::Ptr plotter_;
};

}
#endif // TWC_APPLICATION_APPLICATION_H
