#ifndef TWC_APPLICATION_CARTESIAN_EXAMPLE_H
#define TWC_APPLICATION_CARTESIAN_EXAMPLE_H

#include <Eigen/Geometry>
#include <tesseract_msgs/GetMotionPlanAction.h>
#include <tesseract_command_language/core/serialization.h>
#include <tesseract_command_language/command_language.h>

namespace twc
{
inline tesseract_msgs::GetMotionPlanGoal createCartesianExampleGoal()
{
  tesseract_msgs::GetMotionPlanGoal goal;
  goal.request.name = goal.request.DESCARTES_PLANNER_NAME;

  tesseract_planning::ManipulatorInfo manip_info("robot_only");
  manip_info.tcp = tesseract_planning::ToolCenterPoint("st_tool0");
  tesseract_planning::CompositeInstruction program("RASTER_ROBOT", tesseract_planning::CompositeInstructionOrder::ORDERED, manip_info);

  // Start Joint Position for the program
  std::vector<std::string> joint_names = { "robot_joint_1", "robot_joint_2", "robot_joint_3",
                                           "robot_joint_4", "robot_joint_5", "robot_joint_6" };
  tesseract_planning::StateWaypoint swp1(joint_names, Eigen::VectorXd::Zero(6));
  tesseract_planning::PlanInstruction start_instruction(swp1, tesseract_planning::PlanInstructionType::START);
  program.setStartInstruction(start_instruction);

  tesseract_planning::CartesianWaypoint cwp1(Eigen::Isometry3d::Identity() * Eigen::Translation3d(-0.2, 0, 1.5) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d(1, 0, 0)));
  tesseract_planning::CartesianWaypoint cwp2(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.2, 0, 1.5) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d(1, 0, 0)));
  program.push_back(tesseract_planning::PlanInstruction(cwp1, tesseract_planning::PlanInstructionType::FREESPACE, "FREESPACE_ROBOT"));
  program.push_back(tesseract_planning::PlanInstruction(cwp2, tesseract_planning::PlanInstructionType::LINEAR, "RASTER_ROBOT"));

  goal.request.instructions = tesseract_planning::Serialization::toArchiveStringXML(program);
  return goal;
}
}
#endif // TWC_APPLICATION_CARTESIAN_EXAMPLE_H
