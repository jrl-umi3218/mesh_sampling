

#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <iostream>
#include <iterator>
#include <vector>
#include <stdexcept>
#include <mesh_sampling/assimp_scene.h>
#include <mesh_sampling/qhull_io.h>
#include <mesh_sampling/weighted_random_sampling.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>

namespace fs = std::filesystem;

namespace mesh_sampling
{

const auto supported_cloud_type = std::vector<std::string>{"xyz", "xyz_rgb", "xyz_normal", "xyz_rgb_normal"};
const auto supported_extensions = std::vector<std::string>{".ply", ".pcd", ".qc", ".stl"};

class MeshSampling
{
public:
  MeshSampling(const fs::path & in_path, float scale = 1);

  template<typename PointT>
  void create_cloud(unsigned N, const fs::path & out_path, bool binary_mode = false);

  void convertTo(const fs::path & out_path, bool binary = false);

  bool check_supported(const std::vector<std::string> & supported, const std::string & value);

  /** Get mesh at index 
   * 
   * \param index mesh index (default 0)
   * @return ASSIMPScene& 
   */
  inline ASSIMPScene & mesh(size_t index = 0)
  {
    if(index > meshes_.size())
      throw std::out_of_range("index > meshes.size()");

    return *meshes_.at(index);
  }

private:
  std::vector<std::unique_ptr<ASSIMPScene>> meshes_;
  float scale_;
};

template<typename PointT>
void MeshSampling::create_cloud(unsigned N, const fs::path & out_path, bool binary_mode)
{
  for(const auto &mesh : meshes_)
  {
    WeightedRandomSampling<PointT> sampler(mesh->scene());
    auto cloud = sampler.weighted_random_sampling(N);

    auto out = out_path.string();
    auto extension = boost::algorithm::to_lower_copy(out_path.extension().string());
    bool success = true;
    if(extension == ".pcd")
    {
      success = pcl::io::savePCDFile(out, *cloud, binary_mode) == 0;
    }
    else if(extension == ".ply")
    {
      success = pcl::io::savePLYFile(out, *cloud, binary_mode) == 0;
    }
    else if(extension == ".qc")
    {
      success = mesh_sampling::io::saveQhullFile(out, *cloud);
    }
    else if(extension == ".stl")
    {
      pcl::ConvexHull<PointT> convex{};
      const auto shared_cloud = typename pcl::PointCloud<PointT>::ConstPtr(cloud.release());
      convex.setInputCloud(shared_cloud);
      pcl::PolygonMesh mesh;
      convex.reconstruct(mesh);
      pcl::PointCloud<PointT> cloud{};
      pcl::fromPCLPointCloud2(mesh.cloud, cloud);

      // Compute the convex center
      Eigen::Vector3f center = Eigen::Vector3f::Zero();
      for(auto & poly : mesh.polygons)
      {
        if(poly.vertices.size() != 3)
        {
          throw std::runtime_error("pcl::ConvexHull did not reconstruct a triangular mesh");
        }
        center += (cloud[poly.vertices[0]].getVector3fMap() + cloud[poly.vertices[1]].getVector3fMap()
                   + cloud[poly.vertices[2]].getVector3fMap())
                  / (3 * mesh.polygons.size());
      }

      // Make sure all normals point away from the mesh center
      // Since the mesh is convex this is enough to ensure the normals are consistent
      for(auto & poly : mesh.polygons)
      {
        Eigen::Vector3f p1 = cloud[poly.vertices[0]].getVector3fMap();
        Eigen::Vector3f p2 = cloud[poly.vertices[1]].getVector3fMap();
        Eigen::Vector3f p3 = cloud[poly.vertices[2]].getVector3fMap();
        Eigen::Vector3f n = (p2 - p1).cross(p3 - p1);
        if(n.dot(center - p1) > 0)
        {
          std::swap(poly.vertices[1], poly.vertices[2]);
        }
      }
#if PCL_VERSION_COMPARE(>=, 1, 8, 0)
      success = pcl::io::savePolygonFileSTL(out, mesh, binary_mode);
#else
      success = pcl::io::savePolygonFileSTL(out, mesh);
#endif
    }
    else
    {
      std::cerr << "Output pointcloud type " << extension << " is not supported";
      return;
    }

    if(!success)
    {
      std::cerr << "Saving to " << out << " failed." << std::endl;
      return;
    }
  }
}

}; // namespace mesh_sampling