#include <ros/ros.h>
#include <ros/package.h>
#include <tf2_eigen/tf2_eigen.h>

#include <tesseract_rosutils/plotting.h>
#include <tesseract_visualization/visualization_loader.h>
#include <tesseract_monitoring/environment_monitor_interface.h>

#include <twc_application/raster_applicataion.h>

using tesseract_environment::Environment;
using tesseract_monitoring::EnvironmentMonitorInterface;

static const std::string TOOLPATH = "twc_toolpath";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "application_node");
  ros::NodeHandle nh, pnh("~");

  // Create a tesseract interface
  EnvironmentMonitorInterface interface("tesseract_environment");
  interface.addNamespace("tesseract_workcell_environment");
  if (!interface.wait())
  {
    ROS_ERROR("The monitor namespace 'tesseract_workcell_environment' is not available!");
    return 0;
  }

  Environment::Ptr env = interface.getEnvironment("tesseract_workcell_environment");
  auto current_transforms = env->getState().link_transforms;

  // Dynamically load ignition visualizer if exist
  tesseract_visualization::VisualizationLoader loader;
  auto plotter = loader.get();

  if (plotter != nullptr && env != nullptr)
  {
    plotter->waitForConnection(3);
    plotter->plotEnvironment(*env);
  }

  if((plotter != nullptr && !plotter->isConnected()) || (plotter == nullptr && env != nullptr))
  {
    plotter = std::make_shared<tesseract_rosutils::ROSPlotting>();
    plotter->waitForConnection(3);
    plotter->plotEnvironment(*env);
  }

  plotter->waitForInput("Hit enter to run application!");

  // Create Application
  twc::RasterApplication app(env, plotter);

  // Run application
  app.run();



  ////////////////////////////////////////////////////////////////////////
  // Now lets use the results and set as seed then just plan with trajopt
  ////////////////////////////////////////////////////////////////////////
//  if (result->response.successful)
//  {
//    goal.request.seed = result->response.results;
//    goal.request.name = "RasterTrajOpt";

//    ac.sendGoal(goal);
//    ac.waitForResult();

//    actionlib::SimpleClientGoalState seed_state = ac.getState();
//    ROS_INFO("Action (With Seed) finished: %s", seed_state.toString().c_str());

//    result = ac.getResult();
//    Instruction seed_program_results = Serialization::fromArchiveStringXML<Instruction>(result->response.results);

//    if (!result->response.successful)
//    {
//      ROS_ERROR("Get Motion Plan Failed: %s", result->response.status_string.c_str());
//    }
//    else
//    {
//      ROS_ERROR("Get Motion Plan Successful!");
//    }

//    if (plotter != nullptr && env != nullptr)
//    {
//      const auto& ci = seed_program_results.as<CompositeInstruction>();

////      plotter->waitForInput();
////      plotter->plotToolpath(env->getStateSolver(), seed_program_results);

//      plotter->waitForInput();
//      plotter->plotTrajectory(tesseract_planning::toJointTrajectory(ci), env->getStateSolver());
//    }
//  }
  ros::spin();

  return 0;
}
