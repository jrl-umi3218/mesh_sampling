/*
 * Copyright 2017-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <Eigen/Geometry>
#include <effolkronium/random.hpp>
#include <mesh_sampling/weighted_random_sampling.h>
using Random = effolkronium::random_static;

namespace mesh_sampling
{

double triangle_area(const Eigen::Vector3f & v1, const Eigen::Vector3f & v2, const Eigen::Vector3f & v3)
{
  return 0.5 * (v2 - v1).cross(v3 - v1).norm();
}

/**
 * @brief Use barycentric coordinates to compute random point coordinates in a
 * triangle
 *
 * @param v1,v2,v3 : triangle vertices
 *
 * @return a random point in the triangle (v1,v2,v3)
 */
Eigen::Vector3f random_point_in_triangle(const Eigen::Vector3f & v1,
                                         const Eigen::Vector3f & v2,
                                         const Eigen::Vector3f & v3)
{
  float u = Random::get(0.0f, 1.0f);
  float v = Random::get(0.0f, 1.0f);
  if(u + v > 1)
  {
    u = 1 - u;
    v = 1 - v;
  }
  return (v1 * u) + (v2 * v) + ((1 - (u + v)) * v3);
}

} // namespace mesh_sampling
