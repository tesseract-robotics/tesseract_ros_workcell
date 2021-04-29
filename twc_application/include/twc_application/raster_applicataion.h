#ifndef TWC_APPLICATION_RASTER_APPLICATAION_H
#define TWC_APPLICATION_RASTER_APPLICATAION_H

#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/client/simple_action_client.h>
#include <string>

#include <twc_application/application.h>
#include <twc_application/utils.h>

#include <tesseract_msgs/GetMotionPlanAction.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/conversions.h>
#include <tesseract_command_language/core/serialization.h>
#include <tesseract_command_language/utils/utils.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_visualization/visualization_loader.h>
#include <tesseract_visualization/markers/toolpath_marker.h>
#include <tesseract_monitoring/environment_monitor_interface.h>
#include <tesseract_environment/ofkt/ofkt_state_solver.h>

namespace twc
{

class RasterApplication : public Application
{
public:
  RasterApplication(tesseract_environment::Environment::Ptr env, tesseract_visualization::Visualization::Ptr plotter)
    : Application(std::move(env), std::move(plotter))
    , ac_("/twc_planning_server/tesseract_get_motion_plan", true)
  {
    ros::NodeHandle nh, pnh("~");
    toolpath_ = ros::package::getPath("twc_application") + "/config/job_path.yaml";
    toolpath_ = pnh.param<std::string>("toolpath", toolpath_);

    // Get global parameter
    use_rail_ = nh.param<bool>("/use_rail", use_rail_);
  }

  void run() override
  {
    // wait for the action server to start
    ROS_INFO("Waiting for action server to start.");
    ac_.waitForServer();  // will wait for infinite time

    auto current_transforms = env_->getCurrentState()->link_transforms;

    Eigen::Isometry3d post_transform = Eigen::Isometry3d::Identity() * Eigen::Translation3d(0, 0, 0.05) *
                                       Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()) *
                                       Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
    Eigen::Isometry3d pre_transform = current_transforms["part_link"];

    tesseract_common::Toolpath paths = parsePathFromFile(toolpath_, pre_transform, post_transform);
    tesseract_common::Toolpath raster_strips = filterPath(paths);

    std::string motion_group;
    if (use_rail_)
      motion_group = "manipulator";
    else
      motion_group = "robot_only";

    tesseract_planning::ManipulatorInfo manip_info(motion_group);
    manip_info.tcp = tesseract_planning::ToolCenterPoint("st_tool0");

    tesseract_planning::CompositeInstruction program = createProgram(manip_info, raster_strips);

    // Plot Tool Path
    plotToolpath(program);

    // Create motion planning goal request
    tesseract_msgs::GetMotionPlanGoal goal;
    goal.request.name = goal.request.RASTER_G_FT_PLANNER_NAME;
    goal.request.instructions = tesseract_planning::Serialization::toArchiveStringXML<tesseract_planning::Instruction>(program);

    // Send goal
    ac_.sendGoal(goal);
    ac_.waitForResult();

    actionlib::SimpleClientGoalState state = ac_.getState();
    ROS_INFO("Action finished: %s", state.toString().c_str());

    auto result = ac_.getResult();
    tesseract_planning::Instruction program_results = tesseract_planning::Serialization::fromArchiveStringXML<tesseract_planning::Instruction>(result->response.results);

    if (!result->response.successful)
    {
      ROS_ERROR("Raster Application Motion Plan Failed: %s", result->response.status_string.c_str());
    }
    else
    {
      ROS_ERROR("Raster Application Motion Plan Successful!");
      plotTrajectory(program_results);
    }
  }

  tesseract_planning::CompositeInstruction createProgram(const tesseract_planning::ManipulatorInfo& manip_info, const tesseract_common::Toolpath& raster_strips)
  {

    std::string raster_profile {"RASTER_ROBOT"};
    std::string transition_profile {"TRANSITION_ROBOT"};
    std::string freespace_profile {"FREESPACE_ROBOT"};
    std::vector<std::string> joint_names { "robot_joint_1", "robot_joint_2", "robot_joint_3", "robot_joint_4", "robot_joint_5", "robot_joint_6" };
    if (use_rail_)
    {
      raster_profile = "RASTER_RAIL";
      transition_profile = "TRANSITION_RAIL";
      freespace_profile = "FREESPACE_RAIL";
      joint_names = { "rail_joint_1", "robot_joint_1", "robot_joint_2", "robot_joint_3", "robot_joint_4", "robot_joint_5", "robot_joint_6" };
    }

    tesseract_planning::CompositeInstruction program("raster_program", tesseract_planning::CompositeInstructionOrder::ORDERED, manip_info);

    tesseract_planning::StateWaypoint swp1(joint_names, Eigen::VectorXd::Zero(joint_names.size()));
    tesseract_planning::PlanInstruction start_instruction(swp1, tesseract_planning::PlanInstructionType::START, freespace_profile);
    program.setStartInstruction(start_instruction);

    for (std::size_t rs = 0; rs < raster_strips.size(); ++rs)
    {
      if (rs == 0)
      {
        // Define from start composite instruction
        tesseract_planning::CartesianWaypoint wp1 = raster_strips[rs][0];
        tesseract_planning::PlanInstruction plan_f0(wp1, tesseract_planning::PlanInstructionType::FREESPACE, freespace_profile);
        plan_f0.setDescription("from_start_plan");
        tesseract_planning::CompositeInstruction from_start(freespace_profile);
        from_start.setDescription("from_start");
        from_start.push_back(plan_f0);
        program.push_back(from_start);
      }

      // Define raster
      tesseract_planning::CompositeInstruction raster_segment(raster_profile);
      raster_segment.setDescription("Raster #" + std::to_string(rs + 1));

      for (std::size_t i = 1; i < raster_strips[rs].size(); ++i)
      {
        tesseract_planning::CartesianWaypoint wp = raster_strips[rs][i];
        raster_segment.push_back(tesseract_planning::PlanInstruction(wp, tesseract_planning::PlanInstructionType::LINEAR, raster_profile));
      }
      program.push_back(raster_segment);


      if (rs < raster_strips.size() - 1)
      {
        // Add transition
        tesseract_planning::CartesianWaypoint twp = raster_strips[rs + 1].front();

        tesseract_planning::PlanInstruction tranisiton_instruction1(twp, tesseract_planning::PlanInstructionType::FREESPACE, transition_profile);
        tranisiton_instruction1.setDescription("transition_from_end_plan");

        tesseract_planning::CompositeInstruction transition(transition_profile);
        transition.setDescription("transition_from_end");
        transition.push_back(tranisiton_instruction1);

        program.push_back(transition);
      }
      else
      {
        // Add to end instruction
        tesseract_planning::PlanInstruction plan_f2(swp1, tesseract_planning::PlanInstructionType::FREESPACE, freespace_profile);
        plan_f2.setDescription("to_end_plan");
        tesseract_planning::CompositeInstruction to_end(freespace_profile);
        to_end.setDescription("to_end");
        to_end.push_back(plan_f2);
        program.push_back(to_end);
      }
    }

    return program;
  }

private:
  std::string toolpath_;
  bool use_rail_ {false};
  actionlib::SimpleActionClient<tesseract_msgs::GetMotionPlanAction> ac_;
};

}
#endif // TWC_APPLICATION_RASTER_APPLICATAION_H
