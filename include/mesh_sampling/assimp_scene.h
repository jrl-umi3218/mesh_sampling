#pragma once

#include <boost/algorithm/string.hpp>

#include <assimp/Exporter.hpp>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <filesystem>
#include <stdexcept>
namespace fs = std::filesystem;

namespace mesh_sampling
{

struct ASSIMPScene
{
  ASSIMPScene(const std::string & model_path) : modelPath_(model_path)
  {
    loadScene();
  }

  ASSIMPScene(const std::string & model_path, float scale) : modelPath_(model_path)
  {
    importer.SetPropertyFloat(AI_CONFIG_GLOBAL_SCALE_FACTOR_KEY, scale);
    loadScene();
  }

  /**
   * @brief Do not store the returned pointer, only use it
   *
   * @return
   */
  const aiScene * scene() const
  {
    return scene_;
  }

  /**
   * @brief Export the scene to a file in a format supported by ASSIMP
   *
   * @param path Export path
   */
  void exportScene(const std::string & path, const bool binary = true)
  {
    fs::path out_path(path);
    auto ext = boost::algorithm::to_lower_copy(out_path.extension().string());
    if(ext.empty())
    {
      throw std::runtime_error("Could't export scene " + modelPath_ + " to " + path + ": invalid extension");
    }
    ext.erase(0, 1); // remove leading "."
    if(binary) ext += "b"; // ASSIMP suffixes binary export formats with "b"
    Assimp::Exporter exporter;
    aiReturn ret = exporter.Export(scene_, ext, path);
    if(ret != AI_SUCCESS)
    {
      throw std::runtime_error("ASSIMP failed to export scene " + modelPath_ + " to " + path + ": "
                               + exporter.GetErrorString());
    }
  }

protected:
  void loadScene()
  {
    scene_ =
        importer.ReadFile(modelPath_, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_GenNormals
                                          | aiProcess_FixInfacingNormals | aiProcess_GlobalScale);

    // If the import failed, report it
    if(!scene_)
    {
      throw std::runtime_error(importer.GetErrorString());
    }
  }

protected:
  // The importer will automatically delete the scene
  Assimp::Importer importer;
  const aiScene * scene_;
  std::string modelPath_;
};

} // namespace mesh_sampling
