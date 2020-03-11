#ifndef TWC_MOTION_PLANNING_PLANNING_MANAGER_H
#define TWC_MOTION_PLANNING_PLANNING_MANAGER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PoseArray.h>

#include <twc_msgs/ProcessJobAction.h>
#include <tesseract/tesseract.h>
#include <tesseract_monitoring/environment_monitor.h>

namespace twc_motion_planning
{
class PlanningManager
{
public:
  PlanningManager(ros::NodeHandle nh, tesseract::Tesseract::Ptr tesseract_ptr, std::string robot_id);
  virtual ~PlanningManager() = default;

private:
  /** @brief This constructs the process job problem and solves it using a Tesseract Planners*/
  void jobExecuteCB(const twc_msgs::ProcessJobGoalConstPtr& goal);

  /** @brief Currently this is mostly empty.*/
  void jobPreemptCB();

  /**
   * @brief The the tcp transformation for the provided process type
   * @param tcp The returned tcp
   * @param process_type The process type to lookup tcp for.
   * @return False if unable to find links associated with the process type in the environment, otherwise true
   */
  bool getTCP(Eigen::Isometry3d& tcp, const std::string& base_link, const std::string& tip_link) const;

  ros::NodeHandle nh_;
  tesseract_monitoring::EnvironmentMonitorPtr env_monitor_;
  tesseract::Tesseract::Ptr tesseract_;

  actionlib::SimpleActionServer<twc_msgs::ProcessJobAction> job_as_;

  ros::Publisher posearray_pub_;
  ros::Publisher posearray_pub2_;

  std::string robot_id_;
};

}  // namespace twc_motion_planning
#endif  // TWC_MOTION_PLANNING_PLANNING_MANAGER_H
