/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <gtest/gtest.h>
#include <iostream>
#include <limits>
#include <memory>
#include <mesh_sampling/assimp_scene.h>
#include <mesh_sampling/mesh_sampling.h>
#include <mesh_sampling/weighted_random_sampling.h>

#ifndef PROJECT_SOURCE_DIR
#  error "PROJECT_SOURCE_DIR must be defined to compile this file"
#endif

using namespace mesh_sampling;

// This is a very simple test that merely loads a sample CAD model,
// and samples N point from it.
// It only tests that the object can be loaded, and that the number of points
// after sampling is as expected
TEST(TestWeightedRandomSampling, Test1)
{
  const std::string model_path = PROJECT_SOURCE_DIR "/sample/suzanne.obj";
  size_t N = 1000;

  ASSERT_THROW(ASSIMPScene("wrong path.dae"), std::runtime_error);
  MeshSampling sampler(model_path);

  std::cout << "Sampling " << N << " points from " << model_path << std::endl;
  WeightedRandomSampling sampling_xyz(sampler.mesh(model_path)->scene());
  auto cloud_xyz = sampling_xyz.weighted_random_sampling(N);
  // pcl::io::savePCDFileASCII("/tmp/example_xyz.pcd", *cloud_xyz);
  ASSERT_TRUE(cloud_xyz->size() == N);

  WeightedRandomSampling sampling_rgb(sampler.mesh(model_path)->scene());
  auto cloud_rgb = sampling_rgb.weighted_random_sampling(N);
  ASSERT_TRUE(cloud_rgb->size() == N);

  WeightedRandomSampling sampling_normal(sampler.mesh(model_path)->scene());
  auto cloud_normal = sampling_normal.weighted_random_sampling(N);
  ASSERT_TRUE(cloud_normal->size() == N);

  WeightedRandomSampling sampling_rgb_normal(sampler.mesh(model_path)->scene());
  auto cloud_rgb_normal = sampling_rgb_normal.weighted_random_sampling(N);
  ASSERT_TRUE(cloud_rgb_normal->size() == N);
}

// Sampling with a per-axis scale gives points of the scaled mesh. Suzanne is not symmetric in y, so the test also
// checks that the negative scale mirrors the mesh
TEST(TestWeightedRandomSampling, Scale)
{
  const std::string model_path = PROJECT_SOURCE_DIR "/sample/suzanne.obj";
  const Eigen::Vector3f scale(2.f, -1.f, 0.5f);
  MeshSampling sampler(model_path);
  const aiScene * scene = sampler.mesh(model_path)->scene();

  // Bounding box of the scaled mesh
  Eigen::Vector3f lower = Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
  Eigen::Vector3f upper = -lower;
  for(unsigned int i = 0; i < scene->mNumMeshes; ++i)
  {
    const aiMesh * mesh = scene->mMeshes[i];
    for(unsigned int j = 0; j < mesh->mNumVertices; ++j)
    {
      const auto & v = mesh->mVertices[j];
      const Eigen::Vector3f p = scale.cwiseProduct(Eigen::Vector3f(v.x, v.y, v.z));
      lower = lower.cwiseMin(p);
      upper = upper.cwiseMax(p);
    }
  }

  const size_t N = 10000;
  const auto cloud = sampler.create_cloud(scene, N, {}, false, scale);
  ASSERT_EQ(cloud.size(), N);

  // All the points are in the box, and they reach its sides
  Eigen::Vector3f cloud_lower = Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
  Eigen::Vector3f cloud_upper = -cloud_lower;
  for(const auto & p : cloud)
  {
    ASSERT_TRUE((p.array() >= lower.array() - 1e-4f).all() && (p.array() <= upper.array() + 1e-4f).all());
    cloud_lower = cloud_lower.cwiseMin(p);
    cloud_upper = cloud_upper.cwiseMax(p);
  }
  const Eigen::Vector3f tolerance = 0.02f * (upper - lower);
  EXPECT_TRUE(((cloud_lower - lower).array() <= tolerance.array()).all());
  EXPECT_TRUE(((upper - cloud_upper).array() <= tolerance.array()).all());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
