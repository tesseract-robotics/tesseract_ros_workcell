#include <ros/ros.h>
#include <twc_motion_planning/planning_manager.h>
#include <twc_msgs/ProcessJobAction.h>
#include <geometry_msgs/PoseArray.h>
#include <tesseract_msgs/Trajectory.h>
#include <actionlib/client/simple_action_client.h>
#include <tesseract_rosutils/conversions.h>

#include <yaml-cpp/yaml.h>
#include <yaml-cpp/node/iterator.h>
#include <yaml-cpp/node/impl.h>

#include <tf2_eigen/tf2_eigen.h>

static const std::string TOOLPATH = "twc_toolpath";
static const std::vector<std::string> HOME_JOINTS = { "positioner_joint_1", "positioner_joint_2", "robot_joint_1",
                                                      "robot_joint_2",      "robot_joint_3",      "robot_joint_4",
                                                      "robot_joint_5",      "robot_joint_6" };
static const std::vector<double> HOME_POSITION = { 0, 0, 0, -0.349066, 0.349066, 0, 0, 0 };

///
/// \brief parsePathFromFile Creates a collection of raster strips from a yaml file
/// \param yaml_filepath
/// \param waypoint_origin_frame
/// \return success
///
bool parsePathFromFile(std::vector<geometry_msgs::PoseArray>& raster_strips,
                       const std::string& yaml_filepath,
                       const std::string& waypoint_origin_frame)
{
  std::vector<geometry_msgs::PoseArray> temp_raster_strips;
  YAML::Node full_yaml_node = YAML::LoadFile(yaml_filepath);
  YAML::Node paths = full_yaml_node[0]["paths"];
  std::double_t offset_strip = 0.0;
  for (YAML::const_iterator path_it = paths.begin(); path_it != paths.end(); ++path_it)
  {
    std::vector<geometry_msgs::PoseStamped> temp_poses;
    geometry_msgs::PoseArray curr_pose_array;
    YAML::Node strip = (*path_it)["poses"];
    for (YAML::const_iterator pose_it = strip.begin(); pose_it != strip.end(); ++pose_it)
    {
      const YAML::Node& pose = *pose_it;
      try
      {
        geometry_msgs::PoseStamped current_pose;

        float x = pose["position"]["x"].as<float>();
        float y = pose["position"]["y"].as<float>();
        float z = pose["position"]["z"].as<float>();

        float qx = pose["orientation"]["x"].as<float>();
        float qy = pose["orientation"]["y"].as<float>();
        float qz = pose["orientation"]["z"].as<float>();
        float qw = pose["orientation"]["w"].as<float>();

        current_pose.pose.position.x = x;
        current_pose.pose.position.y = y;
        current_pose.pose.position.z = z;

        current_pose.pose.orientation.x = qx;
        current_pose.pose.orientation.y = qy;
        current_pose.pose.orientation.z = qz;
        current_pose.pose.orientation.w = qw;

        current_pose.header.frame_id = waypoint_origin_frame;

        std::double_t offset_waypoint = offset_strip;

        Eigen::Isometry3d original_pose;
        tf2::fromMsg(current_pose.pose, original_pose);

        Eigen::Isometry3d offset_pose =
            original_pose * Eigen::Translation3d(0.0, 0.0, offset_waypoint) * Eigen::Quaterniond(0, 1, 0, 0);

        current_pose.pose = tf2::toMsg(offset_pose);
        curr_pose_array.poses.push_back(current_pose.pose);
        curr_pose_array.header = current_pose.header;

        temp_poses.push_back(current_pose);
      }
      catch (YAML::InvalidNode& e)
      {
        continue;
      }
    }
    temp_raster_strips.push_back(curr_pose_array);
  }
  raster_strips.reserve(temp_raster_strips.size());
  raster_strips = temp_raster_strips;
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "application_node");
  ros::NodeHandle nh, pnh("~");

  std::string tool_path = "/home/larmstrong/catkin_ws/trajopt_only_ws/src/tesseract-1/tesseract_ros/tesseract_examples/"
                          "tesseract_ros_workcell/twc_application/config/job_path.yaml";
  pnh.param<std::string>("tool_path", tool_path);
  ROS_INFO("Using tool path file: %s", tool_path.c_str());

  // Trajectory publisher
  ros::Publisher trajectory_pub =
      nh.advertise<tesseract_msgs::Trajectory>("/twc_motion_planning/process_trajectory", 1, true);

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<twc_msgs::ProcessJobAction> ac("plan_job", true);

  // wait for the action server to start
  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer();  // will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  twc_msgs::ProcessJobGoal goal;

  //  goal.start_state.name = { "axis_1", "axis_2", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };
  //  goal.start_state.position = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  goal.start_state.name = { "robot_joint_1", "robot_joint_2", "robot_joint_3",
                            "robot_joint_4", "robot_joint_5", "robot_joint_6" };
  goal.start_state.position = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  goal.end_state = goal.start_state;

  goal.header.frame_id = "part_link";
  std::vector<geometry_msgs::PoseArray> paths;
  parsePathFromFile(paths, tool_path, goal.header.frame_id);
  for (std::size_t i = 0; i < paths.size(); ++i)
  {
    if (i < 2)
    {
      goal.paths.push_back(paths[i]);
    }
    else if (i < 5)
    {
      std::vector<geometry_msgs::PoseArray> sub_path(4);
      for (std::size_t j = 0; j < paths[i].poses.size(); ++j)
      {
        if (j < 2)
          sub_path[0].poses.push_back(paths[i].poses[j]);
        else if (j > 7 && j < 12)
          sub_path[1].poses.push_back(paths[i].poses[j]);
        else if (j > 17 && j < 22)
          sub_path[2].poses.push_back(paths[i].poses[j]);
        else if (j > paths[i].poses.size() - 3)
          sub_path[3].poses.push_back(paths[i].poses[j]);
      }
      for (auto& sp : sub_path)
        goal.paths.push_back(sp);
    }
    else
    {
      goal.paths.push_back(paths[i]);
    }
  }

  ac.sendGoal(goal);
  ac.waitForResult();

  actionlib::SimpleClientGoalState state = ac.getState();
  ROS_INFO("Action finished: %s", state.toString().c_str());
  auto result = ac.getResult();

  // Publish Trajectory for visualization
  tesseract_msgs::Trajectory trajectory_msg;
  tesseract_rosutils::toJointTrajectory(trajectory_msg.joint_trajectory, result->process_plans[0]);
  trajectory_pub.publish(trajectory_msg);

  ros::spin();

  return 0;
}
