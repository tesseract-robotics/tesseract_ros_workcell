#ifndef TWC_APPLICATION_RASTER_EXAMPLE_H
#define TWC_APPLICATION_RASTER_EXAMPLE_H

#include <Eigen/Geometry>
#include <tesseract_msgs/GetMotionPlanAction.h>
#include <tesseract_command_language/core/serialization.h>
#include <tesseract_command_language/command_language.h>

#include <yaml-cpp/yaml.h>
#include <yaml-cpp/node/iterator.h>
#include <yaml-cpp/node/impl.h>

namespace twc
{

///
/// \brief parsePathFromFile Creates a collection of raster strips from a yaml file
/// \param yaml_filepath
/// \return success
///
inline tesseract_common::Toolpath parsePathFromFile(const std::string& yaml_filepath,
                                                    const Eigen::Isometry3d& pre_transform = Eigen::Isometry3d::Identity(),
                                                    const Eigen::Isometry3d& post_transform = Eigen::Isometry3d::Identity())
{
  tesseract_common::Toolpath raster_strips;

  YAML::Node full_yaml_node = YAML::LoadFile(yaml_filepath);
  YAML::Node paths = full_yaml_node[0]["paths"];
  for (YAML::const_iterator path_it = paths.begin(); path_it != paths.end(); ++path_it)
  {
    tesseract_common::VectorIsometry3d temp_poses;
    YAML::Node strip = (*path_it)["poses"];
    for (YAML::const_iterator pose_it = strip.begin(); pose_it != strip.end(); ++pose_it)
    {
      const YAML::Node& pose = *pose_it;
      try
      {
        Eigen::Isometry3d current_pose;

        double x = pose["position"]["x"].as<double>();
        double y = pose["position"]["y"].as<double>();
        double z = pose["position"]["z"].as<double>();

        double qx = pose["orientation"]["x"].as<double>();
        double qy = pose["orientation"]["y"].as<double>();
        double qz = pose["orientation"]["z"].as<double>();
        double qw = pose["orientation"]["w"].as<double>();

        current_pose.translation() = Eigen::Vector3d(x, y, z);
        current_pose.linear() = Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();

        temp_poses.push_back(pre_transform * current_pose * post_transform);
      }
      catch (YAML::InvalidNode& e)
      {
        continue;
      }
    }
    raster_strips.push_back(temp_poses);
  }
  return raster_strips;
}

inline tesseract_common::Toolpath filterPath(tesseract_common::Toolpath& paths)
{
  tesseract_common::Toolpath filter_paths;

  for (std::size_t i = 0; i < paths.size(); ++i)
  {

    if (i < 2)
    {
      filter_paths.push_back(paths[i]);
    }
    else if (i < 5)
    {
      tesseract_common::VectorIsometry3d path;
      for (std::size_t j = 0; j < paths[i].size(); ++j)
      {
        if (j < 2)
          path.push_back(paths[i][j]);
        else if (j > 7 && j < 12)
          path.push_back(paths[i][j]);
        else if (j > 17 && j < 22)
          path.push_back(paths[i][j]);
        else if (j > paths[i].size() - 3)
          path.push_back(paths[i][j]);
      }

      filter_paths.push_back(path);
    }
    else
    {
      filter_paths.push_back(paths[i]);
    }
  }

  return filter_paths;
}

inline tesseract_msgs::GetMotionPlanGoal createRasterExampleGoal(std::string tool_path,
                                                                 Eigen::Isometry3d tcp,
                                                                 tesseract_common::TransformMap current_transforms)
{
  tesseract_msgs::GetMotionPlanGoal goal;
  goal.request.name = goal.request.RASTER_G_FT_PLANNER_NAME;

  // Calculate offset
  Eigen::Isometry3d post_transform = Eigen::Isometry3d::Identity() * Eigen::Translation3d(0, 0, 0.05) *
                                     Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()) *
                                     Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
  Eigen::Isometry3d pre_transform = current_transforms["part_link"];

  tesseract_common::Toolpath paths = parsePathFromFile(tool_path, pre_transform, post_transform);
  tesseract_common::Toolpath raster_strips = filterPath(paths);

  tesseract_planning::ManipulatorInfo manip_info("robot_only");
  manip_info.tcp = tesseract_planning::ToolCenterPoint("st_tool0");
  tesseract_planning::CompositeInstruction program("raster_program", tesseract_planning::CompositeInstructionOrder::ORDERED, manip_info);

  // Start Joint Position for the program
  std::vector<std::string> joint_names = { "robot_joint_1", "robot_joint_2", "robot_joint_3",
                                           "robot_joint_4", "robot_joint_5", "robot_joint_6" };
  tesseract_planning::StateWaypoint swp1(joint_names, Eigen::VectorXd::Zero(6));
  tesseract_planning::PlanInstruction start_instruction(swp1, tesseract_planning::PlanInstructionType::START, "FREESPACE_ROBOT");
  program.setStartInstruction(start_instruction);

  for (std::size_t rs = 0; rs < raster_strips.size(); ++rs)
  {
    if (rs == 0)
    {
      // Define from start composite instruction
      tesseract_planning::CartesianWaypoint wp1 = raster_strips[rs][0];
      tesseract_planning::PlanInstruction plan_f0(wp1, tesseract_planning::PlanInstructionType::FREESPACE, "FREESPACE_ROBOT");
      plan_f0.setDescription("from_start_plan");
      tesseract_planning::CompositeInstruction from_start;
      from_start.setDescription("from_start");
      from_start.push_back(plan_f0);
      program.push_back(from_start);
    }

    // Define raster
    tesseract_planning::CompositeInstruction raster_segment;
    raster_segment.setDescription("Raster #" + std::to_string(rs + 1));

    for (std::size_t i = 1; i < raster_strips[rs].size(); ++i)
    {
      tesseract_planning::CartesianWaypoint wp = raster_strips[rs][i];
      raster_segment.push_back(tesseract_planning::PlanInstruction(wp, tesseract_planning::PlanInstructionType::LINEAR, "RASTER_ROBOT"));
    }
    program.push_back(raster_segment);


    if (rs < raster_strips.size() - 1)
    {
      // Add transition
      tesseract_planning::CartesianWaypoint twp = raster_strips[rs + 1].front();

      tesseract_planning::PlanInstruction tranisiton_instruction1(twp, tesseract_planning::PlanInstructionType::FREESPACE, "TRANSITION_ROBOT");
      tranisiton_instruction1.setDescription("transition_from_end_plan");

      tesseract_planning::CompositeInstruction transition;
      transition.setDescription("transition_from_end");
      transition.push_back(tranisiton_instruction1);

      program.push_back(transition);
    }
    else
    {
      // Add to end instruction
      tesseract_planning::PlanInstruction plan_f2(swp1, tesseract_planning::PlanInstructionType::FREESPACE, "FREESPACE_ROBOT");
      plan_f2.setDescription("to_end_plan");
      tesseract_planning::CompositeInstruction to_end;
      to_end.setDescription("to_end");
      to_end.push_back(plan_f2);
      program.push_back(to_end);
    }
  }

  goal.request.instructions = tesseract_planning::Serialization::toArchiveStringXML<tesseract_planning::Instruction>(program);
  return goal;
}

}

#endif // TWC_APPLICATION_RASTER_EXAMPLE_H
