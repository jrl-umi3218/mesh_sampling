#include <exception>
#include <mesh_sampling/mesh_sampling.h>

using namespace mesh_sampling;

MeshSampling::MeshSampling(const fs::path & in_path, float scale) : scale_(scale)
{
  try
  {
    if(fs::is_directory(in_path))
    {
      for(auto const & dir_entry : std::filesystem::directory_iterator{in_path})
        meshes_.push_back(std::make_unique<ASSIMPScene>(dir_entry.path(), scale_));
    }
    else
    {
      meshes_.push_back(std::make_unique<ASSIMPScene>(in_path, scale_));
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
    for(auto &mesh : meshes_)
        mesh->exportScene(out_path, binary);
}