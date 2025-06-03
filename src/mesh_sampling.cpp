#include <mesh_sampling/mesh_sampling.h>

using namespace mesh_sampling;

MeshSampling::MeshSampling(const fs::path & in_path, float scale) : scale_(scale)
{
  load(in_path, scale);
}

void MeshSampling::load(const fs::path & in_path, float scale)
{
  try
  {
    if(fs::is_directory(in_path))
    {
      for(const auto & dir_entry : std::filesystem::directory_iterator{in_path})
        meshes_.insert(
            std::make_pair(dir_entry.path(), std::make_shared<ASSIMPScene>(dir_entry.path().string(), scale)));
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

bool MeshSampling::check_supported(const std::vector<std::string> & supported, const std::string & value)
{
  if(std::find(supported.begin(), supported.end(), value) == supported.end())
  {
    std::cerr << "This program only supports: ";
    std::copy(supported.begin(), supported.end(), std::ostream_iterator<std::string>(std::cerr, ", "));
    std::cerr << std::endl;
    return false;
  }

  return true;
}

void MeshSampling::convertTo(const fs::path & out_path, bool binary)
{
  for(auto & mesh : meshes_) mesh.second->exportScene(out_path, binary);
}