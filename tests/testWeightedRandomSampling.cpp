/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <algorithm>
#include <filesystem>
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

// Per-axis scaling is baked into the loaded Assimp scene. Suzanne is not symmetric in y, so the test also checks that
// the negative scale mirrors the mesh.
TEST(TestWeightedRandomSampling, Scale)
{
  const std::string model_path = PROJECT_SOURCE_DIR "/sample/suzanne.obj";
  const Eigen::Vector3f scale(2.f, -1.f, 0.5f);
  ASSIMPScene original(model_path);
  MeshSampling sampler(model_path, scale);
  const aiScene * scene = sampler.mesh(model_path)->scene();

  ASSERT_EQ(scene->mNumMeshes, original.scene()->mNumMeshes);

  Eigen::Vector3f lower = Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
  Eigen::Vector3f upper = -lower;
  for(unsigned int i = 0; i < scene->mNumMeshes; ++i)
  {
    const aiMesh * mesh = scene->mMeshes[i];
    const aiMesh * original_mesh = original.scene()->mMeshes[i];
    ASSERT_EQ(mesh->mNumVertices, original_mesh->mNumVertices);
    ASSERT_EQ(mesh->mNumFaces, original_mesh->mNumFaces);

    for(unsigned int j = 0; j < mesh->mNumVertices; ++j)
    {
      const auto & original_vertex = original_mesh->mVertices[j];
      const Eigen::Vector3f expected_vertex =
          scale.cwiseProduct(Eigen::Vector3f(original_vertex.x, original_vertex.y, original_vertex.z));
      const auto & vertex = mesh->mVertices[j];
      EXPECT_TRUE(Eigen::Vector3f(vertex.x, vertex.y, vertex.z).isApprox(expected_vertex, 1e-5f));
      lower = lower.cwiseMin(expected_vertex);
      upper = upper.cwiseMax(expected_vertex);

      if(original_mesh->HasNormals())
      {
        const auto & original_normal = original_mesh->mNormals[j];
        Eigen::Vector3f expected_normal(original_normal.x / scale.x(), original_normal.y / scale.y(),
                                        original_normal.z / scale.z());
        expected_normal.normalize();
        const auto & normal = mesh->mNormals[j];
        EXPECT_TRUE(Eigen::Vector3f(normal.x, normal.y, normal.z).isApprox(expected_normal, 1e-5f));
      }
    }

    for(unsigned int j = 0; j < mesh->mNumFaces; ++j)
    {
      const aiFace & face = mesh->mFaces[j];
      const aiFace * original_face = nullptr;
      std::vector<unsigned int> face_indices(face.mIndices, face.mIndices + face.mNumIndices);
      std::sort(face_indices.begin(), face_indices.end());
      for(unsigned int k = 0; k < original_mesh->mNumFaces; ++k)
      {
        const aiFace & candidate = original_mesh->mFaces[k];
        if(face.mNumIndices != candidate.mNumIndices)
        {
          continue;
        }

        std::vector<unsigned int> candidate_indices(candidate.mIndices, candidate.mIndices + candidate.mNumIndices);
        std::sort(candidate_indices.begin(), candidate_indices.end());
        if(face_indices == candidate_indices)
        {
          original_face = &candidate;
          break;
        }
      }

      ASSERT_NE(original_face, nullptr);
      for(unsigned int k = 0; k < face.mNumIndices; ++k)
      {
        EXPECT_EQ(face.mIndices[k], original_face->mIndices[face.mNumIndices - 1 - k]);
      }
    }
  }

  const size_t N = 10000;
  const auto cloud = sampler.create_cloud(scene, N);
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

TEST(TestWeightedRandomSampling, RejectZeroScale)
{
  const std::string model_path = PROJECT_SOURCE_DIR "/sample/suzanne.obj";
  EXPECT_THROW(MeshSampling(model_path, Eigen::Vector3f(1.f, 0.f, 1.f)), std::invalid_argument);
}

TEST(TestMeshSampling, CreateCloudFromSingleMeshPath)
{
  const std::filesystem::path model_path = PROJECT_SOURCE_DIR "/sample/suzanne.obj";
  constexpr unsigned N = 1000;
  MeshSampling sampler(model_path);

  const auto cloud = sampler.create_cloud(model_path, N);

  ASSERT_EQ(cloud.size(), N);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
