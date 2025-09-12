/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <gtest/gtest.h>
#include <iostream>
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

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
