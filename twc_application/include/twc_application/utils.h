#ifndef UTILS_H
#define UTILS_H

#include <Eigen/Geometry>
#include <tesseract_common/types.h>

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
}
#endif // UTILS_H
