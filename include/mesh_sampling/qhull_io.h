/*
 * Copyright 2017-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once
#include <Eigen/Core>
#include <fstream>
#include <vector>
namespace mesh_sampling
{

using CloudT = std::vector<Eigen::Vector3f>;
namespace io
{

bool saveQhullFile(const std::string & path, const CloudT & cloud)
{
  std::ofstream file;
  file.open(path, std::ios::out);
  if(!file.is_open())
  {
    return false;
  }

  file << "3" << std::endl;
  file << std::to_string(cloud.size()) << std::endl;
  for(const auto & point : cloud)
  {
    file << point.x() << " " << point.y() << " " << point.z() << std::endl;
  }

  return true;
}

} // namespace io
} // namespace mesh_sampling
