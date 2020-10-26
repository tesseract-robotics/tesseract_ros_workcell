#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseArray.h>
#include <tesseract_msgs/GetMotionPlanAction.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/conversions.h>
#include <actionlib/client/simple_action_client.h>
#include <tesseract_command_language/deserialize.h>
#include <tesseract_visualization/visualization_loader.h>
#include <tesseract_monitoring/tesseract_monitor_interface.h>
#include <tesseract_environment/ofkt/ofkt_state_solver.h>

#include <twc_application/cartesian_example.h>
#include <twc_application/freespace_example.h>
#include <twc_application/raster_example.h>

#include <tf2_eigen/tf2_eigen.h>

using namespace tesseract_planning;

static const std::string TOOLPATH = "twc_toolpath";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "application_node");
  ros::NodeHandle nh, pnh("~");

  std::string tool_path = ros::package::getPath("twc_application") + "/config/job_path.yaml";
  pnh.param<std::string>("tool_path", tool_path);
  ROS_INFO("Using tool path file: %s", tool_path.c_str());

  // Create a tesseract interface
  tesseract_monitoring::TesseractMonitorInterface interface("tesseract_environment");
  interface.addNamespace("tesseract_workcell_environment");
  if (!interface.wait())
  {
    ROS_ERROR("The monitor namespace 'tesseract_workcell_environment' is not avialable!");
    return 0;
  }

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

  // Get TCP
  Eigen::Isometry3d tcp = current_transforms["robot_tool0"].inverse() * current_transforms["st_tool0"];

  ROS_INFO("Action server started, sending goal.");
  tesseract_msgs::GetMotionPlanGoal goal = twc::createCartesianExampleGoal(tcp);
//  tesseract_msgs::GetMotionPlanGoal goal = twc::createFreespaceExampleGoal(tcp);
//  tesseract_msgs::GetMotionPlanGoal goal = twc::createRasterExampleGoal(tool_path, tcp, current_transforms);

  // Plot Tool Path
  if (plotter != nullptr && thor != nullptr)
  {
    plotter->waitForInput();
    plotter->plotToolPath(fromXMLString(goal.request.instructions));
  }

  // Send goal
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

  ////////////////////////////////////////////////////////////////////////
  // Now lets use the results and set as seed then just plan with trajopt
  ////////////////////////////////////////////////////////////////////////
  if (result->response.successful)
  {
    goal.request.seed = result->response.results;
    goal.request.name = goal.TRAJOPT_PLANNER_NAME;

    ac.sendGoal(goal);
    ac.waitForResult();

    actionlib::SimpleClientGoalState seed_state = ac.getState();
    ROS_INFO("Action (With Seed) finished: %s", seed_state.toString().c_str());

    result = ac.getResult();
    Instruction seed_program_results = fromXMLString(result->response.results);

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
      plotter->plotToolPath(seed_program_results);

      plotter->waitForInput();
      plotter->plotTrajectory(seed_program_results);
    }
  }
  ros::spin();

  return 0;
}
