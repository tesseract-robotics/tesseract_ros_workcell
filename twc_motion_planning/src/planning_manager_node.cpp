#include <ros/ros.h>
#include <twc_motion_planning/planning_manager.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract/tesseract.h>

static const std::string ROBOT_DESCRIPTION_PARAM =
    "robot_description"; /**< Default ROS parameter for robot description */
static const std::string ROBOT_SEMANTIC_PARAM =
    "robot_description_semantic"; /**< Default ROS parameter for robot description */

int main(int argc, char** argv)
{
  ros::init(argc, argv, "planning_manager_node");
  ros::NodeHandle nh, pnh("~");

  std::string urdf_xml_string, srdf_xml_string, robot_id;
  nh.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
  nh.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);
  pnh.param<std::string>("robot_id", robot_id, "TWC");

  // Initialize the environment
  tesseract::Tesseract::Ptr tesseract_ptr = std::make_shared<tesseract::Tesseract>();
  auto locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  if (!tesseract_ptr->init(urdf_xml_string, srdf_xml_string, locator))
  {
    ROS_ERROR("Planning Manager: Failed to load URDF and SRDF");
    return -1;
  }

  twc_motion_planning::PlanningManager manager(nh, tesseract_ptr, robot_id);
  ros::spin();

  return 0;
}
