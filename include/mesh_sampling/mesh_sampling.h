#pragma once

#include <Eigen/Core>
#include <filesystem>
#include <map>
#include <vector>

class aiScene;

namespace fs = std::filesystem;

namespace mesh_sampling
{
struct ASSIMPScene;

using CloudT = std::vector<Eigen::Vector3f>;

const auto supported_cloud_type = std::vector<std::string>{"xyz", "xyz_rgb", "xyz_normal", "xyz_rgb_normal"};
const auto supported_extensions = std::vector<std::string>{".ply", ".qc", ".stl", ".obj", ".dae"};

class MeshSampling
{
public:
  MeshSampling() = default;
  /**
   * @brief Constructs a MeshSampling object from the given file path.
   *
   * Loads a mesh from the specified file and applies the given scale.
   *
   * @param in_path Path to the mesh file to load.
   * @param scale Scaling factor to apply to the mesh (default: Eigen::Vector3f::Ones()).
   *
   * @throws std::runtime_error If the mesh loading fails.
   */
  MeshSampling(const fs::path & in_path, const Eigen::Vector3f & scale = Eigen::Vector3f::Ones());
  ~MeshSampling() = default;

  /**
   * @brief Load mesh from full path or directory
   *
   * @param in_path path or directory
   * @param scale per-axis scale applied to mesh vertices during import (default: no scaling)
   * @throws if the mesh cannot be loaded
   */
  void load(const fs::path & in_path, const Eigen::Vector3f & scale = Eigen::Vector3f::Ones());

  /**
   * @brief Create a cloud
   *
   * @tparam PointT
   * @param N number of sampling points (default: 2000)
   * @return pcl::PointCloud<PointT>
   */
  CloudT cloud(const unsigned N = 2000);

  /**
   * @brief Create and save a pointcloud
   *
   * @tparam PointT
   * @param mesh file path
   * @param N number sampling points
   * @param out_path if empty or directory, file is not saved (default: {})
   * @param binary_mode is binary (default: false)
   * @return pcl::PointCloud<PointT>
   */
  CloudT create_cloud(const fs::path & meshFile, unsigned N, const fs::path & out_path = {}, bool binary_mode = false);

  /**
   * @brief Create and save a pointcloud
   *
   * @tparam PointT
   * @param scene ASSIMPScene
   * @param N number sampling points
   * @param out_path if empty or directory, file is not saved (default: {})
   * @param binary_mode is binary (default: false)
   * @return pcl::PointCloud<PointT>
   */
  CloudT create_cloud(const aiScene * scene, unsigned N, const fs::path & out_path = {}, bool binary_mode = false);

  /**
   * @brief Create and save pointclouds, to be used when importing meshes from a directory. If out_path is empty clouds
   * are not saved
   *
   * @tparam PointT
   * @param N number of sampling points
   * @param out_path directory to save the clouds (default: {})
   * @param extension saving extension (default: ".qc")
   * @param binary_mode is binary mnode (default: false)
   */
  std::map<std::string, CloudT> create_clouds(unsigned N,
                                              const fs::path & out_path = {},
                                              const std::string & extension = ".qc",
                                              bool binary_mode = false);

  /**
   * @brief Create a convex object using qhull library and save it to a file
   *
   * @tparam PointT
   * @param cloud PCL point cloud
   * @param out_path output directory
   */
  std::string create_convex(const CloudT & cloud, const fs::path & out_path = {});

  std::map<std::string, std::string> create_convexes(const std::map<std::string, CloudT> & clouds,
                                                     const fs::path & out_path = {},
                                                     bool stop_on_fail = true);

  /**
   * @brief Export all meshes to a file in a format support by ASSIMP
   *
   * @param out_path Export path
   * @param binary is binary (defaukt:false)
   */
  void convertTo(const fs::path & out_path, bool binary = false);

  /**
   * @brief Check supported type
   *
   * @param type type to check
   * @param supported list of supported types (default : "xyz", "xyz_rgb", "xyz_normal", "xyz_rgb_normal")
   * @return bool
   */
  bool check_supported(const std::string & type, const std::vector<std::string> & supported = supported_cloud_type);

  /** Get mesh at index
   *
   * \param path mesh path
   * @return ASSIMPScene&
   */
  ASSIMPScene * mesh(const std::string & path);

private:
  class Impl;
  std::map<std::string, std::shared_ptr<ASSIMPScene>> meshes_;
};

}; // namespace mesh_sampling
