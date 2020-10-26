#ifndef TWC_APPLICATION_FREESPACE_EXAMPLE_H
#define TWC_APPLICATION_FREESPACE_EXAMPLE_H

#include <Eigen/Geometry>
#include <tesseract_msgs/GetMotionPlanAction.h>
#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/serialize.h>

namespace twc
{
inline tesseract_msgs::GetMotionPlanGoal createFreespaceExampleGoal(Eigen::VectorXd jp, Eigen::Isometry3d tcp)
{
  tesseract_msgs::GetMotionPlanGoal goal;
//  goal.request.name = goal.FREESPACE;

  tesseract_planning::ManipulatorInfo manip_info("robot_only");
  manip_info.tcp = tcp;
  tesseract_planning::CompositeInstruction program("raster_program", tesseract_planning::CompositeInstructionOrder::ORDERED, manip_info);

  // Start Joint Position for the program
  std::vector<std::string> joint_names = { "robot_joint_1", "robot_joint_2", "robot_joint_3",
                                           "robot_joint_4", "robot_joint_5", "robot_joint_6" };
  tesseract_planning::StateWaypoint swp1(joint_names, Eigen::VectorXd::Zero(6));
  tesseract_planning::PlanInstruction start_instruction(swp1, tesseract_planning::PlanInstructionType::START);
  program.setStartInstruction(start_instruction);


  tesseract_planning::JointWaypoint jwp(joint_names, jp);
  program.push_back(tesseract_planning::PlanInstruction(jwp, tesseract_planning::PlanInstructionType::FREESPACE));

  goal.request.instructions = toXMLString(program);
  goal.request.num_threads = 1;
  return goal;
}
}
#endif // TWC_APPLICATION_FREESPACE_EXAMPLE_H
