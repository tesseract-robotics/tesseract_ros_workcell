#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseArray.h>
#include <tesseract_msgs/GetMotionPlanAction.h>
#include <tesseract_rosutils/plotting.h>
#include <actionlib/client/simple_action_client.h>
#include <tesseract_rosutils/conversions.h>
#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/serialize.h>
#include <tesseract_command_language/deserialize.h>
#include <tesseract_visualization/visualization_loader.h>
#include <tesseract_monitoring/tesseract_monitor_interface.h>
#include <tesseract_environment/ofkt/ofkt_state_solver.h>

#include <yaml-cpp/yaml.h>
#include <yaml-cpp/node/iterator.h>
#include <yaml-cpp/node/impl.h>

#include <tf2_eigen/tf2_eigen.h>

using namespace tesseract_planning;

static const std::string TOOLPATH = "twc_toolpath";
static const std::vector<std::string> HOME_JOINTS = { "positioner_joint_1", "positioner_joint_2", "robot_joint_1",
                                                      "robot_joint_2",      "robot_joint_3",      "robot_joint_4",
                                                      "robot_joint_5",      "robot_joint_6" };
static const std::vector<double> HOME_POSITION = { 0, 0, 0, -0.349066, 0.349066, 0, 0, 0 };

///
/// \brief parsePathFromFile Creates a collection of raster strips from a yaml file
/// \param yaml_filepath
/// \return success
///
bool parsePathFromFile(std::vector<std::vector<Eigen::Isometry3d>>& raster_strips,
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

std::vector<std::vector<Eigen::Isometry3d>> filterPath(std::vector<std::vector<Eigen::Isometry3d>>& paths)
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

CompositeInstruction createProgram(const std::vector<std::vector<Eigen::Isometry3d>>& raster_strips,
                                   const std::string& working_frame,
                                   Eigen::Isometry3d tcp,
                                   Eigen::Isometry3d transform = Eigen::Isometry3d::Identity())
{
  ManipulatorInfo manip_info("robot_only");
  manip_info.working_frame = working_frame;
  manip_info.tcp = tcp;
  CompositeInstruction program("raster_program", CompositeInstructionOrder::ORDERED, manip_info);

  // Start Joint Position for the program
  std::vector<std::string> joint_names = { "robot_joint_1", "robot_joint_2", "robot_joint_3",
                                           "robot_joint_4", "robot_joint_5", "robot_joint_6" };
  StateWaypoint swp1 = StateWaypoint(joint_names, Eigen::VectorXd::Zero(6));
  PlanInstruction start_instruction(swp1, PlanInstructionType::START);
  program.setStartInstruction(start_instruction);

  for (std::size_t rs = 0; rs < raster_strips.size(); ++rs)
  {
    if (rs == 0)
    {
      // Define from start composite instruction
      CartesianWaypoint wp1 = transform * raster_strips[rs][0];
      PlanInstruction plan_f0(wp1, PlanInstructionType::FREESPACE, "FREESPACE");
      plan_f0.setDescription("from_start_plan");
      CompositeInstruction from_start;
      from_start.setDescription("from_start");
      from_start.push_back(plan_f0);
      program.push_back(from_start);
    }

    // Define raster
    CompositeInstruction raster_segment;
    raster_segment.setDescription("Raster #" + std::to_string(rs + 1));

    for (std::size_t i = 1; i < raster_strips[rs].size(); ++i)
    {
      CartesianWaypoint wp = transform * raster_strips[rs][i];
      raster_segment.push_back(PlanInstruction(wp, PlanInstructionType::LINEAR, "RASTER"));
    }
    program.push_back(raster_segment);


    if (rs < raster_strips.size() - 1)
    {
      // Add transition
      CartesianWaypoint twp = transform * raster_strips[rs + 1].front();

      PlanInstruction tranisiton_instruction1(twp, PlanInstructionType::FREESPACE, "TRANSITION");
      tranisiton_instruction1.setDescription("transition_from_end_plan");

      CompositeInstruction transition;
      transition.setDescription("transition_from_end");
      transition.push_back(tranisiton_instruction1);

      program.push_back(transition);
    }
    else
    {
      // Add to end instruction
      PlanInstruction plan_f2(swp1, PlanInstructionType::FREESPACE, "FREESPACE");
      plan_f2.setDescription("to_end_plan");
      CompositeInstruction to_end;
      to_end.setDescription("to_end");
      to_end.push_back(plan_f2);
      program.push_back(to_end);
    }
  }

  return program;
}

CompositeInstruction createFreespaceProgram(Eigen::VectorXd jp, Eigen::Isometry3d tcp)
{
  ManipulatorInfo manip_info("robot_only");
  manip_info.tcp = tcp;
  CompositeInstruction program("raster_program", CompositeInstructionOrder::ORDERED, manip_info);

  // Start Joint Position for the program
  std::vector<std::string> joint_names = { "robot_joint_1", "robot_joint_2", "robot_joint_3",
                                           "robot_joint_4", "robot_joint_5", "robot_joint_6" };
  StateWaypoint swp1 = StateWaypoint(joint_names, Eigen::VectorXd::Zero(6));
  PlanInstruction start_instruction(swp1, PlanInstructionType::START);
  program.setStartInstruction(start_instruction);


  JointWaypoint jwp = JointWaypoint(joint_names, jp);
  program.push_back(PlanInstruction(jwp, PlanInstructionType::FREESPACE));
  return program;
}

CompositeInstruction createCartesianProgram(Eigen::Isometry3d tcp)
{
  ManipulatorInfo manip_info("robot_only");
  manip_info.tcp = tcp;
  CompositeInstruction program("raster_program", CompositeInstructionOrder::ORDERED, manip_info);

  // Start Joint Position for the program
  std::vector<std::string> joint_names = { "robot_joint_1", "robot_joint_2", "robot_joint_3",
                                           "robot_joint_4", "robot_joint_5", "robot_joint_6" };
  StateWaypoint swp1 = StateWaypoint(joint_names, Eigen::VectorXd::Zero(6));
  PlanInstruction start_instruction(swp1, PlanInstructionType::START);
  program.setStartInstruction(start_instruction);

  CartesianWaypoint cwp1(Eigen::Isometry3d::Identity() * Eigen::Translation3d(-0.2, 0, 1.5) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d(1, 0, 0)));
  CartesianWaypoint cwp2(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.2, 0, 1.5) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d(1, 0, 0)));
  program.push_back(PlanInstruction(cwp1, PlanInstructionType::FREESPACE));
  program.push_back(PlanInstruction(cwp2, PlanInstructionType::LINEAR));
  return program;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "application_node");
  ros::NodeHandle nh, pnh("~");

  std::string tool_path = ros::package::getPath("twc_application") + "/config/job_path.yaml";
  pnh.param<std::string>("tool_path", tool_path);
  ROS_INFO("Using tool path file: %s", tool_path.c_str());

  // Create a tesseract interface
  tesseract_monitoring::TesseractMonitorInterface interface("tesseract_workcell");
  tesseract::Tesseract::Ptr thor = interface.getTesseract<tesseract_environment::OFKTStateSolver>("tesseract_workcell_environment");
  auto current_transforms = thor->getEnvironment()->getCurrentState()->link_transforms;

  // Dynamically load ignition visualizer if exist
  tesseract_visualization::VisualizationLoader loader;
  auto plotter = loader.get();

  if (plotter != nullptr && thor != nullptr)
  {
    plotter->init(thor);
    plotter->waitForConnection(3);
    plotter->plotEnvironment();
  }

  if((plotter != nullptr && !plotter->isConnected()) || (plotter == nullptr && thor != nullptr))
  {
    plotter = std::make_shared<tesseract_rosutils::ROSPlotting>();
    plotter->init(thor);
    plotter->waitForConnection(3);
  }

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<tesseract_msgs::GetMotionPlanAction> ac("/twc_planning_server/tesseract_get_motion_plan", true);

  // wait for the action server to start
  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer();  // will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  tesseract_msgs::GetMotionPlanGoal goal;

  Eigen::Isometry3d tcp = current_transforms["robot_tool0"].inverse() * current_transforms["st_tool0"];

  Eigen::Isometry3d rot_offset = Eigen::Isometry3d::Identity() * Eigen::Translation3d(0, 0, 0.05) * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d(0, 0, 1));
  Eigen::Isometry3d transform = current_transforms["part_link"];
  std::vector<std::vector<Eigen::Isometry3d>> paths;
  parsePathFromFile(paths, tool_path);
  std::vector<std::vector<Eigen::Isometry3d>> filtered_path = filterPath(paths);
  CompositeInstruction program = createProgram(filtered_path, "", tcp * rot_offset, transform);
  goal.request.name = goal.RASTER_G_FT_PLANNER_NAME;

//  Eigen::VectorXd jp = Eigen::VectorXd::Zero(6);
//  jp(0) = M_PI_2;
//  CompositeInstruction program = createFreespaceProgram(jp, tcp * rot_offset);
//  goal.request.name = goal.CARTESIAN_PLANNER_NAME;

//  Eigen::Isometry3d rot_offset = Eigen::Isometry3d::Identity() * Eigen::Translation3d(0, 0, 0.05) * Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d(0, 0, 1));
//  CompositeInstruction program = createCartesianProgram(tcp * rot_offset);
//  goal.request.name = goal.DESCARTES_PLANNER_NAME;

  if (plotter != nullptr && thor != nullptr)
  {
    plotter->waitForInput();
    plotter->plotToolPath(program);
  }

  goal.request.instructions = toXMLString(program);
  goal.request.num_threads = 1;

  ac.sendGoal(goal);
  ac.waitForResult();

  actionlib::SimpleClientGoalState state = ac.getState();
  ROS_INFO("Action finished: %s", state.toString().c_str());

  auto result = ac.getResult();
  Instruction program_results = fromXMLString(result->response.results);

  if (!result->response.successful)
  {
    ROS_ERROR("Get Motion Plan Failed: %s", result->response.status_string.c_str());
  }
  else
  {
    ROS_ERROR("Get Motion Plan Successful!");
  }

  if (plotter != nullptr && thor != nullptr)
  {
    plotter->waitForInput();
    plotter->plotToolPath(program_results);

    plotter->waitForInput();
    plotter->plotTrajectory(program_results);
  }

  ros::spin();

  return 0;
}
