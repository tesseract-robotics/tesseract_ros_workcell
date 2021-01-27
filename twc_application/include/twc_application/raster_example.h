#ifndef TWC_APPLICATION_RASTER_EXAMPLE_H
#define TWC_APPLICATION_RASTER_EXAMPLE_H

#include <Eigen/Geometry>
#include <tesseract_msgs/GetMotionPlanAction.h>
#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/serialize.h>

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
inline bool parsePathFromFile(std::vector<std::vector<Eigen::Isometry3d>>& raster_strips,
                              const std::string& yaml_filepath)
{
  YAML::Node full_yaml_node = YAML::LoadFile(yaml_filepath);
  YAML::Node paths = full_yaml_node[0]["paths"];
  std::double_t offset_strip = 0.0;
  for (YAML::const_iterator path_it = paths.begin(); path_it != paths.end(); ++path_it)
  {
    std::vector<Eigen::Isometry3d> temp_poses;
    YAML::Node strip = (*path_it)["poses"];
    for (YAML::const_iterator pose_it = strip.begin(); pose_it != strip.end(); ++pose_it)
    {
      const YAML::Node& pose = *pose_it;
      try
      {
        Eigen::Isometry3d current_pose;

        float x = pose["position"]["x"].as<float>();
        float y = pose["position"]["y"].as<float>();
        float z = pose["position"]["z"].as<float>();

        float qx = pose["orientation"]["x"].as<float>();
        float qy = pose["orientation"]["y"].as<float>();
        float qz = pose["orientation"]["z"].as<float>();
        float qw = pose["orientation"]["w"].as<float>();

        current_pose.translation() = Eigen::Vector3d(x, y, z);
        current_pose.linear() = Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();

        Eigen::Isometry3d offset_pose =
            current_pose * Eigen::Translation3d(0.0, 0.0, offset_strip) * Eigen::Quaterniond(0, 1, 0, 0);

        temp_poses.push_back(offset_pose);
      }
      catch (YAML::InvalidNode& e)
      {
        continue;
      }
    }
    raster_strips.push_back(temp_poses);
  }
  return true;
}

inline std::vector<std::vector<Eigen::Isometry3d>> filterPath(std::vector<std::vector<Eigen::Isometry3d>>& paths)
{
  std::vector<std::vector<Eigen::Isometry3d>> filter_paths;

  for (std::size_t i = 0; i < paths.size(); ++i)
  {

    if (i < 2)
    {
      filter_paths.push_back(paths[i]);
    }
    else if (i < 5)
    {
      std::vector<Eigen::Isometry3d> path;
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

  Eigen::Isometry3d rot_offset = Eigen::Isometry3d::Identity() * Eigen::Translation3d(0, 0, 0.05) * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d(0, 0, 1));
  Eigen::Isometry3d transform = current_transforms["part_link"];
  std::vector<std::vector<Eigen::Isometry3d>> paths;
  parsePathFromFile(paths, tool_path);
  std::vector<std::vector<Eigen::Isometry3d>> raster_strips = filterPath(paths);

  tesseract_planning::ManipulatorInfo manip_info("robot_only");
  manip_info.tcp = tcp * rot_offset;
  tesseract_planning::CompositeInstruction program("raster_program", tesseract_planning::CompositeInstructionOrder::ORDERED, manip_info);

  // Start Joint Position for the program
  std::vector<std::string> joint_names = { "robot_joint_1", "robot_joint_2", "robot_joint_3",
                                           "robot_joint_4", "robot_joint_5", "robot_joint_6" };
  tesseract_planning::StateWaypoint swp1(joint_names, Eigen::VectorXd::Zero(6));
  tesseract_planning::PlanInstruction start_instruction(swp1, tesseract_planning::PlanInstructionType::START);
  program.setStartInstruction(start_instruction);

  for (std::size_t rs = 0; rs < raster_strips.size(); ++rs)
  {
    if (rs == 0)
    {
      // Define from start composite instruction
      tesseract_planning::CartesianWaypoint wp1 = transform * raster_strips[rs][0];
      tesseract_planning::PlanInstruction plan_f0(wp1, tesseract_planning::PlanInstructionType::FREESPACE, "FREESPACE");
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
      tesseract_planning::CartesianWaypoint wp = transform * raster_strips[rs][i];
      raster_segment.push_back(tesseract_planning::PlanInstruction(wp, tesseract_planning::PlanInstructionType::LINEAR, "RASTER"));
    }
    program.push_back(raster_segment);


    if (rs < raster_strips.size() - 1)
    {
      // Add transition
      tesseract_planning::CartesianWaypoint twp = transform * raster_strips[rs + 1].front();

      tesseract_planning::PlanInstruction tranisiton_instruction1(twp, tesseract_planning::PlanInstructionType::FREESPACE, "TRANSITION");
      tranisiton_instruction1.setDescription("transition_from_end_plan");

      tesseract_planning::CompositeInstruction transition;
      transition.setDescription("transition_from_end");
      transition.push_back(tranisiton_instruction1);

      program.push_back(transition);
    }
    else
    {
      // Add to end instruction
      tesseract_planning::PlanInstruction plan_f2(swp1, tesseract_planning::PlanInstructionType::FREESPACE, "FREESPACE");
      plan_f2.setDescription("to_end_plan");
      tesseract_planning::CompositeInstruction to_end;
      to_end.setDescription("to_end");
      to_end.push_back(plan_f2);
      program.push_back(to_end);
    }
  }

  goal.request.instructions = tesseract_planning::toXMLString<tesseract_planning::Instruction>(program);
  goal.request.num_threads = 1;
  return goal;
}

}

#endif // TWC_APPLICATION_RASTER_EXAMPLE_H
