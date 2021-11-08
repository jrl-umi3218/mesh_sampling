#pragma once

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#include <assimp/Exporter.hpp>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <sstream>
#include <stdexcept>
namespace bfs = boost::filesystem;

namespace mesh_sampling
{

class ASSIMPScene
{
  // The importer will automatically delete the scene
  Assimp::Importer importer;
  const aiScene * scene_;
  std::string modelPath_;

public:
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

  static std::vector<std::string> supportedExportFormats()
  {
    std::vector<std::string> supportedFormats;
    Assimp::Exporter exporter;
    auto numExporters = exporter.GetExportFormatCount();
    for(auto i = 0u; i < numExporters; i++)
    {
      const aiExportFormatDesc * format = exporter.GetExportFormatDescription(i);
      supportedFormats.push_back(format->fileExtension);
    }
    return supportedFormats;
  }

  /**
   * @brief Export the scene to a file in a format supported by ASSIMP
   * @see supportedExportFormats for a list of supported extensions
   *
   * @param path Export path
   */
  void exportScene(const std::string & path)
  {
    Assimp::Exporter exporter;
    auto numExporters = exporter.GetExportFormatCount();
    bfs::path out_path(path);
    auto ext = boost::algorithm::to_lower_copy(out_path.extension().string());
    if(ext.empty())
    {
      throw std::runtime_error("Could't export scene " + modelPath_ + " to " + path + ": invalid extension");
    }
    ext.erase(0, 1); // remove leading "."
    // Find exporter id for this extension
    int index = -1;
    for(auto i = 0u; i < numExporters; i++)
    {
      const aiExportFormatDesc * format = exporter.GetExportFormatDescription(i);
      if(ext == format->fileExtension)
      {
        index = i;
        break;
      }
    }
    if(index == -1)
    { // Unsupported export format
      std::string s;
      const auto supportedFormats = supportedExportFormats();
      std::ostringstream ss;
      std::copy(supportedFormats.begin(), supportedFormats.end() - 1, std::ostream_iterator<std::string>(ss, ", "));
      throw std::runtime_error("Could't export: unsupported format " + ext + "\nSupported formats are: " + ss.str());
    }
    const aiExportFormatDesc * format = exporter.GetExportFormatDescription(index);
    aiReturn ret = exporter.Export(scene_, format->id, path);
    if(ret != AI_SUCCESS)
    {
      throw std::runtime_error("ASSIMP failed to export scene " + modelPath_ + " to " + path);
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
};

} // namespace mesh_sampling
