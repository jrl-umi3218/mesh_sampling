#include "mesh_sampling/mesh_sampling.h"

// Deactivate clang to ensure include order required by libqhullcpp
// clang-format off
#include <filesystem>
#include <libqhullcpp/Qhull.h>
#include <libqhullcpp/QhullFacetSet.h>
#include <libqhullcpp/QhullLinkedList.h>
#include <libqhullcpp/QhullPoint.h>
#include <libqhullcpp/QhullUser.h>
#include <libqhullcpp/QhullVertexSet.h>
#include <libqhull/io.h>
#include <mesh_sampling/assimp_scene.h>
#include <mesh_sampling/mesh_sampling.h>
#include <mesh_sampling/qhull_io.h>
#include <mesh_sampling/weighted_random_sampling.h>
//clang-format on

using namespace orgQhull;

namespace mesh_sampling
{

MeshSampling::MeshSampling(const fs::path & in_path, float scale)
{
  load(in_path, scale);
}

std::string MeshSampling::create_convex(const CloudT & cloud, const fs::path & out_path)
{
  if(cloud.empty())
  {
    throw std::invalid_argument("create_convex: input cloud is empty.");
  }

  char * buffer = nullptr;
  size_t size = 0;
  FILE * out_stream = open_memstream(&buffer, &size);

  if(out_stream == nullptr)
  {
    throw std::runtime_error("create_convex: failed to open memory stream.");
  }

  // Create Qhull object
  Qhull qhull;

  std::ofstream ofs;
  if(!out_path.empty()){
    ofs.open(out_path.c_str());

    if(!ofs.is_open())
    {
      throw std::invalid_argument("create_convex: could not open file :" + out_path.string());
    }
  }

  qhull.qh()->fout = out_stream;

  // Convert PCL cloud to a flat array for Qhull input
  std::vector<double> qhull_input;
  qhull_input.reserve(cloud.size() * 3);
  for(const auto & pt : cloud)
  {
    qhull_input.push_back(pt.x());
    qhull_input.push_back(pt.y());
    qhull_input.push_back(pt.z());
  }

  try
  {
    qhull.runQhull("pcl_input", 3, cloud.size(), qhull_input.data(), "Qt"); // 3D, triangulate option
    qhull.outputQhull("o f");
  }
  catch(const std::exception & e)
  {
    fclose(out_stream);
    free(buffer);
    throw std::runtime_error(std::string("Qhull run failed: ") + e.what());
  }

  fclose(out_stream);
  std::string output(buffer, size);
  free(buffer);

  if (!out_path.empty()) {
    ofs << output;
    std::cout << "Convex file saved to " << out_path << std::endl;
  }

  return output;
}

std::map<std::string, std::string> MeshSampling::create_convexes(const std::map<std::string, CloudT> & clouds,
                                         const fs::path & out_path,
                                         bool stop_on_fail)
{
  if(!out_path.empty() && !fs::is_directory(out_path))
  {
    throw std::invalid_argument("create_convexes: out_path has to be a directory {" + out_path.string() + "}");
  }

  std::map<std::string, std::string> output;

  for(const auto & cloud : clouds)
  {
    try
    {
      if(out_path.empty())
        output[cloud.first] = create_convex(cloud.second, {});
      else
        output[cloud.first] = create_convex(cloud.second, out_path / (fs::path(cloud.first).filename().stem().string() + "-ch.txt"));
    }
    catch(const std::exception & e)
    {
      std::cerr << e.what() << std::endl;

      if(stop_on_fail) return {};
    }
  }

  return output;
}

std::map<std::string, CloudT> MeshSampling::create_clouds(unsigned N,
                                                                                 const fs::path & out_path,
                                                                                 const std::string & extension,
                                                                                 bool binary_mode)
{
  std::map<std::string, CloudT> clouds;
  if(!out_path.empty())
  {
    if(!fs::is_directory(out_path) && meshes_.size() > 1)
    {
      throw std::runtime_error("[Error] create_clouds, out_path is not a directory");
    }

    if(extension.empty())
    {
      throw std::runtime_error("[Error] create_clouds, extension is not set");
    }

    for(const auto & mesh : meshes_)
    {
      auto path = fs::is_directory(out_path) ? out_path / (fs::path(mesh.first).filename().stem().string() + extension)
                                             : out_path;
      auto cloud = create_cloud(mesh.second->scene(), N, path, binary_mode);
      clouds.insert({mesh.first, cloud});
    }
  }
  else
  {
    for(const auto & mesh : meshes_)
    {
      auto cloud = create_cloud(mesh.second->scene(), N, {}, binary_mode);
      clouds.insert({mesh.first, cloud});
    }
  }

  return clouds;
}

CloudT MeshSampling::cloud(const unsigned N)
{
  WeightedRandomSampling sampler(meshes_.begin()->second->scene());
  auto cloud = sampler.weighted_random_sampling(N);
  return *cloud;
}

CloudT MeshSampling::create_cloud(const aiScene * scene,
                                                         const unsigned N,
                                                         const fs::path & out_path,
                                                         bool binary_mode)
{
  WeightedRandomSampling sampler(scene);
  auto cloud = sampler.weighted_random_sampling(N);

  if(!out_path.empty() && !fs::is_directory(out_path))
  {
    auto extension = out_path.extension().string();
    bool success = false;

    if(!success)
    {
      std::cerr << "Saving to " << out_path << " failed." << std::endl;
      return {};
    }
    else
    {
      std::cout << "Saving cloud to " << out_path << " success" << std::endl;
    }
  }

  return *cloud;
}

void MeshSampling::load(const fs::path & in_path, float scale)
{
  try
  {
    if(fs::is_directory(in_path))
    {
      for(const auto & dir_entry : std::filesystem::directory_iterator{in_path}){
        if(check_supported(dir_entry.path().extension(), supported_extensions)){
          meshes_.insert(
              std::make_pair(dir_entry.path(), std::make_shared<ASSIMPScene>(dir_entry.path(), scale)));
        }
      }
    }
    else
    {
      meshes_.insert(std::make_pair(in_path, std::make_shared<ASSIMPScene>(in_path, scale)));
    }
  }
  catch(std::runtime_error & e)
  {
    std::cerr << e.what() << std::endl;
    return;
  }
}

void MeshSampling::convertTo(const fs::path & out_path, bool binary)
{
  for(auto & mesh : meshes_) mesh.second->exportScene(out_path, binary);
}

bool MeshSampling::check_supported(const std::string & type, const std::vector<std::string> & supported)
{
  return std::any_of(supported.begin(), supported.end(),
                     [&](const std::string & ext) { return std::equal(ext.begin(), ext.end(), type.begin(), type.end(),
                      [](char a, char b) { return std::tolower(a) == std::tolower(b); }); });
}

ASSIMPScene * MeshSampling::mesh(const std::string & path)
  {
    if(meshes_.count(path) > 0) return meshes_.begin()->second.get();

    return nullptr;
  }

} // namespace mesh_sampling
